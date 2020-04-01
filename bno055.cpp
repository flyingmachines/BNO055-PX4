#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <px4_module.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/bno_first.h>

#include <board_config.h>

#define BNO055_BUS_DEFAULT           PX4_I2C_BUS_EXPANSION
#define BNO055_BASEADDR      0x28
#define BNO055_WHO_AM_I_REG  0x00
#define BNO055_WHO_AM_I_REG_VAL 0xA0
#define BNO055_HEADING_LSB_REG  0x1A 
#define BNO055_HEADING_MSB_REG  0x1B
#define BNO055_OPERATION_CONFIG 0x00
#define BNO055_OPERATION_MODE   0x3D
#define BNO055_PWR_MODE 		0x3E
#define BNO055_NORMAL_MODE 		0x00
#define BNO055_SYS_TRIGGER 		0x3F
#define BNO055_PAGE_ID			0x07
#define BNO055_UNIT_SEL 		0x3B
#define BNO055_DEVICE_PATH   	"/dev/bno055"

#define BNO055_CONVERSION_INTERVAL  50000  /*500 ms*/
#define BNO055_CHIP_ID 	0x00

#ifndef CONFIG_SCHED_WORKQUEUE
# error THIS requires CONFIG_SCHED_WORKQUEUE.
#endif

class BNO055 : public device::I2C
{
public:
	BNO055(int bus = BNO055_BUS_DEFAULT, int address = BNO055_BASEADDR);
	virtual ~BNO055();

	virtual int 		init();

	virtual int 		senssetup();

	virtual int 		testdata();

	//virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	//virtual int			probe();

private:
	work_s				_work{};
	ringbuffer::RingBuffer		*_reports;
	int				_measure_ticks;
	bool			_cyclesend;
	int				_class_instance;
	int				_orb_class_instance;


	orb_advert_t		_bno055_sensor_topic;

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults TRONE_MIN_DISTANCE
	* and TRONE_MAX_DISTANCE
	*/

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycleandsend();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void		cycle_trampoline(void *arg);


};

extern "C" __EXPORT int bno055_main(int argc, char *argv[]);

BNO055::BNO055(int bus, int address) :
	I2C("BNO055", BNO055_DEVICE_PATH, bus, address, 100000),
	_reports(nullptr),
	_measure_ticks(0),
	_cyclesend(true),
	_class_instance(-1),
	_orb_class_instance(-1),
	_bno055_sensor_topic(nullptr)
{
	// up the retries since the device misses the first measure attempts
	I2C::_retries = 3;

	memset(&_work, 0, sizeof(_work));
}

BNO055::~BNO055()
{
	// /* make sure we are truly inactive */
	stop();

	// /* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(BNO055_DEVICE_PATH, _class_instance);
	}

	// // free perf counters
	// perf_free(_sample_perf);
	// perf_free(_comms_errors);
}

int
BNO055::init()
{
	int ret = PX4_ERROR;

	set_device_address(BNO055_BASEADDR);

	if (I2C::init() != OK) {

		PX4_ERR("initialization failed");
		goto out;

	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(bno_first_s));

	if (_reports == nullptr) {
		goto out;
	}

	_class_instance = register_class_devname(BNO055_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* get a publish handle on the imu sensor topic */
		struct bno_first_s bno_report;

		_reports->get(&bno_report);

		_bno055_sensor_topic = orb_advertise_multi(ORB_ID(bno_first), &bno_report,
					 &_orb_class_instance, ORB_PRIO_LOW);

		if (_bno055_sensor_topic == nullptr) {
			PX4_ERR("failed to create distance_sensor object");
		}
	}


	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
out:
	return ret;
}

int
BNO055::ioctl()
{
	bool start_poll = (_measure_ticks == 0);

	_measure_ticks = USEC2TICK(BNO055_CONVERSION_INTERVAL);

	if (start_poll){
		start();
	}

	return OK;
}

void
BNO055::start()
{

	work_queue(HPWORK, &_work, (worker_t)&BNO055::cycle_trampoline, this, 1);

}

void 
BNO055::cycle_trampoline(void *arg)
{
	BNO055 *ptr = (BNO055 *)arg;

	ptr->cycleandsend();
}

void
BNO055::stop()
{
	work_cancel(HPWORK, &_work);
}

void
BNO055::cycleandsend()
{


	//Collect the heading data in this section 
	int ret = -EIO;

	uint8_t lsbhead = 0;
	uint8_t msbhead = 0;
	if(_cyclesend){
	
		uint8_t cmd = BNO055_HEADING_MSB_REG;

		if (transfer(&cmd, 1, nullptr, 0) == OK) {
			ret = transfer(nullptr, 0, &msbhead, 1);
			if (ret < 0){
				return;
			}
		}

		cmd = BNO055_HEADING_LSB_REG;

		if (transfer(&cmd, 1, nullptr, 0) == OK) {
			ret = transfer(nullptr, 0, &lsbhead, 1);
			if (ret < 0){
				return;
			}
		}
	
	}

	int16_t heading = (int16_t)((uint16_t)lsbhead | ((uint16_t)msbhead << 8));
	float headingf = heading / 16.0f;

	struct bno_first_s bnreport;
	bnreport.timestamp = hrt_absolute_time();
	bnreport.angle = headingf;
	bnreport.id = 0xa0;

	if ( _bno055_sensor_topic != nullptr) {
		orb_publish(ORB_ID(bno_first), _bno055_sensor_topic, &bnreport);
	}

	_reports->force(&bnreport);

	poll_notify(POLLIN);

	//printf("Heading is 0x%02x\n", heading);
	//PX4_INFO("heading is %0.3f\n",(double)headingf);

	work_queue(HPWORK, &_work, (worker_t)&BNO055::cycle_trampoline, this, USEC2TICK(BNO055_CONVERSION_INTERVAL));

}
int
BNO055::senssetup()
{
	uint8_t who_am_i = 0;

	uint8_t cmd = BNO055_WHO_AM_I_REG;

	// can't use a single transfer as Teraranger needs a bit of time for internal processing
	if (transfer(&cmd, 1, nullptr, 0) == OK) {
		if (transfer(nullptr, 0, &who_am_i, 1) == OK && who_am_i == BNO055_WHO_AM_I_REG_VAL) {
			PX4_INFO("This is the BNO055 sensor 0x%02x\n",(unsigned)who_am_i);
			//return OK;
		}else{

		return PX4_ERROR;
		
		}
	}else{

		return PX4_ERROR;
	
	}

	// The sensor is set in config mode
	uint8_t cmdwr[2] = {BNO055_OPERATION_CONFIG, 0x00};
	transfer(&cmdwr[0], 2, nullptr, 0);
	px4_usleep(500000);

	//The sensor reset happens
	cmdwr[0] = BNO055_SYS_TRIGGER;
	cmdwr[1] = 0x20;
	transfer(&cmdwr[0], 2, nullptr, 0);
	px4_usleep(1000000);

	//Reading Chip ID again
	if (transfer(&cmd, 1, nullptr, 0) == OK) {
		if (transfer(nullptr, 0, &who_am_i, 1) == OK && who_am_i == BNO055_WHO_AM_I_REG_VAL) {
			//printf("This is the BNO055 sensor 0x%02x\n",(unsigned)who_am_i);
			//return OK;
		}else{

		return PX4_ERROR;
		
		}
	}else{

		return PX4_ERROR;
	
	}

	px4_usleep(500000);
	// The sensor is in normal power mode 
	cmdwr[0] = BNO055_PWR_MODE;
	cmdwr[1] = BNO055_NORMAL_MODE;
	transfer(&cmdwr[0], 2, nullptr, 0);
	px4_usleep(100000);

	cmdwr[0] = BNO055_PAGE_ID;
	cmdwr[1] = 0x00;
	transfer(&cmdwr[0], 2, nullptr, 0);
	px4_usleep(100000);
	//Using internal crystal
	cmdwr[0] = BNO055_SYS_TRIGGER;
	cmdwr[1] = 0x00;
	transfer(&cmdwr[0], 2, nullptr, 0);
	px4_usleep(200000);

	//Setting Fusion Mode here
	cmdwr[0] = BNO055_OPERATION_MODE;
	cmdwr[1] = 0x08;
	transfer(&cmdwr[0], 2, nullptr, 0);
	px4_usleep(200000);

	PX4_INFO("Fusion mode set");
	px4_usleep(600000);

	//Set unit to degrees
	cmdwr[0] = BNO055_UNIT_SEL;
	cmdwr[1] = 0x00;
	transfer(&cmdwr[0], 2, nullptr, 0);
	px4_usleep(200000);
	// PX4_DEBUG("WHO_AM_I byte mismatch 0x%02x should be 0x%02x\n",
	// 	  (unsigned)who_am_i,
	// 	  BNO055_WHO_AM_I_REG_VAL);

	return OK;
}

int
BNO055::testdata()
{

	int ret = -EIO;
	
for (int i = 0; i < 50; ++i)
	{
	uint8_t lsbhead = 0;
	uint8_t msbhead = 0;

	uint8_t cmd = BNO055_HEADING_MSB_REG;

	if (transfer(&cmd, 1, nullptr, 0) == OK) {
		ret = transfer(nullptr, 0, &msbhead, 1);
		if (ret < 0){
			break;
		}
	}

	cmd = BNO055_HEADING_LSB_REG;

	if (transfer(&cmd, 1, nullptr, 0) == OK) {
		ret = transfer(nullptr, 0, &lsbhead, 1);
		if (ret < 0){
			break;
		}
	}

	ret = OK;

	int16_t heading = (int16_t)((uint16_t)lsbhead | ((uint16_t)msbhead << 8));
	float headingf = heading / 16.0f;
	//printf("Heading is 0x%02x\n", heading);
	printf("heading is %0.3f\n",(double)headingf);
	px4_usleep(500000);

	}

	return ret;


}


namespace bno055
{

BNO055	*g_dev;

int 	start_bus(int i2c_bus);
int 	stop();
int 	test();
int 	reset();
int 	info();

int
start_bus(int i2c_bus)
{
	int fd = -1;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new BNO055(i2c_bus);


	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(BNO055_DEVICE_PATH, O_RDONLY);

	if (OK != g_dev->senssetup()) {
		goto fail;
	}

	if (OK != g_dev->ioctl()) {
		goto fail;
	}

	if (fd < 0) {
		goto fail;
	}

	PX4_INFO("Opened\n");

	//if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
	//	goto fail;
	//}

	px4_close(fd);
	return PX4_OK;

fail:

	if (fd >= 0) {
		px4_close(fd);
	}

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	//g_dev->print_info();

	return PX4_OK;
}
} //namespace

// static void
// bno055_usage()
// {
// 	PRINT_MODULE_DESCRIPTION(
// 		R"DESCR_STR(
// ### Description

// I2C bus driver for TeraRanger rangefinders.

// The sensor/driver must be enabled using the parameter SENS_EN_TRANGER.

// Setup/usage information: https://docs.px4.io/en/sensor/rangefinders.html#teraranger-rangefinders

// ### Examples

// Start driver on any bus (start on bus where first sensor found).
// $ teraranger start -a
// Start driver on specified bus
// $ teraranger start -b 1
// Stop driver
// $ teraranger stop
// )DESCR_STR");

// 	PRINT_MODULE_USAGE_NAME("teraranger", "driver");
// 	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("start","Start driver");
// 	PRINT_MODULE_USAGE_PARAM_FLAG('a', "Attempt to start driver on all I2C buses (first one found)", true);
// 	PRINT_MODULE_USAGE_PARAM_INT('b', 1, 1, 2000, "Start driver on specific I2C bus", true);
// 	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("stop","Stop driver");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("test","Test driver (basic functional tests)");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("reset","Reset driver");
// 	PRINT_MODULE_USAGE_COMMAND_DESCR("info","Print driver information");



// }

int
bno055_main(int argc, char *argv[])
{
	int myoptind = 1;

	int i2c_bus = BNO055_BUS_DEFAULT;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {

		return bno055::start_bus(i2c_bus);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return bno055::stop();
	}

	// /*
	//  * Test the driver/device.
	//  */
	// if (!strcmp(argv[myoptind], "test")) {
	// 	return teraranger::test();
	// }

	// /*
	//  * Reset the driver.
	//  */
	// if (!strcmp(argv[myoptind], "reset")) {
	// 	return teraranger::reset();
	// }

	/*
	 * Print driver information.
	 */
	// if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
	// 	return teraranger::info();
	// }

//out_error:
//	bno055_usage();
	return 0;
}