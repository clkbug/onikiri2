// 
// Copyright (c) 2005-2008 Kenichi Watanabe.
// Copyright (c) 2005-2008 Yasuhiro Watari.
// Copyright (c) 2005-2008 Hironori Ichibayashi.
// Copyright (c) 2008-2009 Kazuo Horio.
// Copyright (c) 2009-2015 Naruki Kurata.
// Copyright (c) 2005-2015 Ryota Shioya.
// Copyright (c) 2005-2015 Masahiro Goshima.
// 
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 
// 1. The origin of this software must not be misrepresented; you must not
// claim that you wrote the original software. If you use this software
// in a product, an acknowledgment in the product documentation would be
// appreciated but is not required.
// 
// 2. Altered source versions must be plainly marked as such, and must not be
// misrepresented as being the original software.
// 
// 3. This notice may not be removed or altered from any source
// distribution.
// 
// 


#ifndef EMU_STRAIGHT64LINUX_STRAIGHT64LINUX_SYSCALL_CONV_H
#define EMU_STRAIGHT64LINUX_STRAIGHT64LINUX_SYSCALL_CONV_H

#include "Emu/Utility/System/Syscall/SyscallConvIF.h"
#include "Emu/Utility/System/VirtualSystem.h"
#include "Emu/Utility/System/Syscall/Linux32SyscallConv.h"

namespace Onikiri {

    namespace EmulatorUtility {
        class ProcessState;
        class MemorySystem;
        class OpEmulationState;
    }

    namespace STRAIGHT64Linux {

        class STRAIGHT64LinuxSyscallConv : public EmulatorUtility::Linux32SyscallConv
        {
        private:
            STRAIGHT64LinuxSyscallConv();
        public:
            STRAIGHT64LinuxSyscallConv(EmulatorUtility::ProcessState* processState);
            virtual ~STRAIGHT64LinuxSyscallConv();

            // SetArg によって与えられた引数に従ってシステムコールを行う
            virtual void Execute(EmulatorUtility::OpEmulationState* opState);
        protected:

            // arch dependent

            // conversion
            //virtual void write_stat64(u64 dest, const EmulatorUtility::HostStat &src);
            virtual int Get_MAP_ANONYMOUS();
            virtual int Get_MREMAP_MAYMOVE();
            virtual int Get_CLK_TCK();

            virtual u32 OpenFlagTargetToHost(u32 flag);
            virtual void syscall_fstat32(EmulatorUtility::OpEmulationState* opState);
            virtual void syscall_stat32(EmulatorUtility::OpEmulationState* opState);
            virtual void write_stat32(u64 dest, const EmulatorUtility::HostStat &src);
        };

    } // namespace STRAIGHT64Linux
} // namespace Onikiri

#endif
