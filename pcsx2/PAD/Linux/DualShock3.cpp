#include <cerrno>
#include <cstring>
#include <iostream>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/hidraw.h>
#include <linux/limits.h>
#include <sys/ioctl.h>

#include "DualShock3.h"

#define DIGITALS 0x2
#define DIGITALS_SELECT 1
#define DIGITALS_L3 1 << 1
#define DIGITALS_R3 1 << 2
#define DIGITALS_START 1 << 3

#define ANALOG_STICK_LEFT_X 0x6
#define ANALOG_STICK_LEFT_Y 0x7
#define ANALOG_STICK_RIGHT_X 0x8
#define ANALOG_STICK_RIGHT_Y 0x9

#define ANALOG_DPAD_UP 0xe
#define ANALOG_DPAD_RIGHT 0xf
#define ANALOG_DPAD_DOWN 0x10
#define ANALOG_DPAD_LEFT 0x11

#define ANALOG_L2 0x12
#define ANALOG_R2 0x13
#define ANALOG_L1 0x14
#define ANALOG_R1 0x15
#define ANALOG_TRIANGLE 0x16
#define ANALOG_CIRCLE 0x17
#define ANALOG_CROSS 0x18
#define ANALOG_SQUARE 0x19

#define OUTPUT_REPORT_DATA_LENGTH 0x24

#define OUTPUT_RUMBLE_SMALL_MOTOR_DURATION 0x2
#define OUTPUT_RUMBLE_SMALL_MOTOR_ON 0x3
#define OUTPUT_RUMBLE_LARGE_MOTOR_DURATION 0x4
#define OUTPUT_RUMBLE_LARGE_MOTOR_FORCE 0x5

static int ds3_axis_to_pcsx2(uint8_t value)
{
	return (value - 127) * (1 << 8);
}

void EnumerateDualShock3s(std::vector<std::unique_ptr<Device>>& vjoysticks)
{
	int result = 0;

	DIR* dev_dir = opendir("/dev");
	if (!dev_dir)
	{
		std::perror("opendir() failed");
		return;
	}
	struct dirent* entry = nullptr;
	errno = 0;
	while ((entry = readdir(dev_dir)))
	{
		if (!strncmp(entry->d_name, "hidraw", 6))
		{
			char hidraw_path[PATH_MAX] = "";
			if ((result = snprintf(hidraw_path, PATH_MAX, "/dev/%s", entry->d_name)) < 0)
			{
				std::cerr << "snprintf() failed with: " << result << std::endl;
				break;
			}

			int fd = open(hidraw_path, O_RDWR | O_NONBLOCK);
			if (fd < 0)
			{
				if (errno == EPERM) {
					errno = 0;
					continue;
				};

				std::perror("open() failed");
				errno = 0;
				break;
			}

			struct hidraw_devinfo info;
			result = ioctl(fd, HIDIOCGRAWINFO, &info);
			if (result < 0)
			{
				std::perror("ioctl() failed");
				result = close(fd);
				if (result < 0)
				{
					std::perror("close() failed");
				}

				errno = 0;
				break;
			}

			if (info.vendor == 0x054c && info.product == 0x0268 && vjoysticks.size() < 1)
			{
				vjoysticks.push_back(std::unique_ptr<Device>(new DualShock3(fd)));
			}
		}
	}
	if (!entry && errno)
	{
		std::perror("readdir() failed");
	}
	if ((result = closedir(dev_dir)) < 0)
	{
		std::perror("closedir() failed");
	}
}

DualShock3::DualShock3(int fd) : Device()
{
	m_fd = fd;
	m_no_error = true;
}

DualShock3::~DualShock3()
{
	if (close(m_fd) < 0)
	{
		std::perror("close() failed");
                m_no_error = false;
	}
}

const char* DualShock3::GetName()
{
	return "DualShock 3 with pressure sensitive buttons";
}

int DualShock3::GetInput(gamePadValues input)
{
	switch (input)
	{
		case PAD_L2:
			return m_report_data[ANALOG_L2];
		case PAD_R2:
			return m_report_data[ANALOG_R2];
		case PAD_L1:
			return m_report_data[ANALOG_L1];
		case PAD_R1:
			return m_report_data[ANALOG_R1];
		case PAD_TRIANGLE:
			return m_report_data[ANALOG_TRIANGLE];
		case PAD_CIRCLE:
			return m_report_data[ANALOG_CIRCLE];
		case PAD_CROSS:
			return m_report_data[ANALOG_CROSS];
		case PAD_SQUARE:
			return m_report_data[ANALOG_SQUARE];
		case PAD_SELECT:
			return m_report_data[DIGITALS] & DIGITALS_SELECT;
		case PAD_L3:
			return m_report_data[DIGITALS] & DIGITALS_L3;
		case PAD_R3:
			return m_report_data[DIGITALS] & DIGITALS_R3;
		case PAD_START:
			return m_report_data[DIGITALS] & DIGITALS_START;
		case PAD_UP:
			return m_report_data[ANALOG_DPAD_UP];
		case PAD_RIGHT:
			return m_report_data[ANALOG_DPAD_RIGHT];
		case PAD_DOWN:
			return m_report_data[ANALOG_DPAD_DOWN];
		case PAD_LEFT:
			return m_report_data[ANALOG_DPAD_LEFT];
		case PAD_L_UP:
			return ds3_axis_to_pcsx2(m_report_data[ANALOG_STICK_LEFT_Y]);
		case PAD_L_RIGHT:
			return ds3_axis_to_pcsx2(m_report_data[ANALOG_STICK_LEFT_X]);
		case PAD_L_DOWN:
			return ds3_axis_to_pcsx2(m_report_data[ANALOG_STICK_LEFT_Y]);
		case PAD_L_LEFT:
			return ds3_axis_to_pcsx2(m_report_data[ANALOG_STICK_LEFT_X]);
		case PAD_R_UP:
			return ds3_axis_to_pcsx2(m_report_data[ANALOG_STICK_RIGHT_Y]);
		case PAD_R_RIGHT:
			return ds3_axis_to_pcsx2(m_report_data[ANALOG_STICK_RIGHT_X]);
		case PAD_R_DOWN:
			return ds3_axis_to_pcsx2(m_report_data[ANALOG_STICK_RIGHT_Y]);
		case PAD_R_LEFT:
			return ds3_axis_to_pcsx2(m_report_data[ANALOG_STICK_RIGHT_X]);
		default:
			return 0;
	}
}

void DualShock3::UpdateDeviceState()
{
	errno = 0;
	int bytes_read = 0;
	// get the most recent report so there is no input lag
	while (bytes_read != -1)
	{
		bytes_read = read(m_fd, m_report_data, OUTPUT_REPORT_DATA_LENGTH);
		// TODO: handle partial reads.
		if (bytes_read < OUTPUT_REPORT_DATA_LENGTH)
		{
			if (bytes_read > -1)
			{
				std::cerr << "read() only got " << bytes_read << " bytes" << std::endl;
				m_no_error = false;
				return;
			}
		}
	}
	if (errno != EAGAIN)
	{
		std::perror("read() failed");
		m_no_error = false;
	}
}

size_t DualShock3::GetUniqueIdentifier()
{
	return 1;
}

void DualShock3::RumbleWithStrength(unsigned type, float strength)
{
	fprintf(stderr, "type is %d, strength is %f\n", type, strength);
	unsigned char data[OUTPUT_REPORT_DATA_LENGTH] = {
		// double report id, hidraw eats the first one for some reason
		0x01,
		0x01,
		// small motor
		0x00,
		0x00,
		// large motor
		0x00,
		0x00,
		// padding
		0x00,
		0x00,
		0x00,
		0x00,
		// leds
		0x02,
		0xff,
		0x27,
		0x10,
		0x00,
		0x32,
		0xff,
		0x27,
		0x10,
		0x00,
		0x32,
		0xff,
		0x27,
		0x10,
		0x00,
		0x32,
		0xff,
		0x27,
		0x10,
		0x00,
		0x32,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
	};
	switch (type)
	{
		case 0:
			data[OUTPUT_RUMBLE_SMALL_MOTOR_DURATION] = 0x10;
			data[OUTPUT_RUMBLE_SMALL_MOTOR_ON] = strength > 0 ? 0x1 : 0x0;
			break;
		case 1:
			data[OUTPUT_RUMBLE_LARGE_MOTOR_DURATION] = 0x10;
			// for some reason values smaller than 128 result in no rumble
			data[OUTPUT_RUMBLE_LARGE_MOTOR_FORCE] = static_cast<std::uint8_t>(strength * 255);
			fprintf(stderr, "uint8_t value is %d, float is %f\n", data[OUTPUT_RUMBLE_LARGE_MOTOR_FORCE], strength);
			break;
		default:
			return;
	}
	int bytes_written = write(m_fd, data, OUTPUT_REPORT_DATA_LENGTH);
	if (bytes_written < OUTPUT_REPORT_DATA_LENGTH)
	{
		if (bytes_written < 0)
		{
			if (errno == EAGAIN)
			{
				m_no_error = false;
				return;
			}

			std::perror("write() failed");
		}
		else
		{
			// TODO: handle partial writes.
			std::cerr << "write() only got " << bytes_written << " bytes" << std::endl;
		}
		m_no_error = false;
	}
}

void DualShock3::Rumble(unsigned type, unsigned pad)
{
	if (!(g_conf.pad_options[pad].forcefeedback))
	{
		return;
	}
	RumbleWithStrength(type, g_conf.get_ff_intensity() / (1 << 7));
}

bool DualShock3::TestForce(float strength)
{
	RumbleWithStrength(1, strength);
	return true;
}
