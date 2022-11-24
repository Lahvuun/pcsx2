#include <cstdint>

#include "Device.h"

#define INPUT_REPORT_DATA_LENGTH 49

class DualShock3 : public Device
{
public:
	DualShock3(int fd);
	~DualShock3();
	const char* GetName() final;
	int GetInput(gamePadValues input) final;

	void UpdateDeviceState() final;

	size_t GetUniqueIdentifier() final;
	void Rumble(unsigned type, unsigned pad);
	bool TestForce(float strength = 0.60);

protected:
	void RumbleWithStrength(unsigned type, float strength);
	int m_fd = -1;
	std::uint8_t m_report_data[INPUT_REPORT_DATA_LENGTH] = {0};
};

void EnumerateDualShock3s(std::vector<std::unique_ptr<Device>>& vjoysticks);
