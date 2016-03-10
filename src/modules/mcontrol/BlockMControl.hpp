/**
 * @file BlockMControl.hpp
 * @author Matthias Grob <maetugr@gmail.com>
 *
 * Nonlinear Quadrotor Controller (master thesis)
 */

#pragma once
#include <controllib/uorb/blocks.hpp>

class BlockMControl : public control::BlockUorbEnabledAutopilot {
public:
	BlockMControl();
private:
};
