﻿//
// Copyright (c) 2005-2008 Kenichi Watanabe.
// Copyright (c) 2005-2008 Yasuhiro Watari.
// Copyright (c) 2005-2008 Hironori Ichibayashi.
// Copyright (c) 2008-2009 Kazuo Horio.
// Copyright (c) 2009-2015 Naruki Kurata.
// Copyright (c) 2005-2015 Masahiro Goshima.
// Copyright (c) 2005-2018 Ryota Shioya.
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


#ifndef EMU_RISCV64LINUX_RISCV64_OPERATION_H
#define EMU_RISCV64LINUX_RISCV64_OPERATION_H

#include "SysDeps/fenv.h"
#include "Utility/RuntimeError.h"
#include "Emu/Utility/GenericOperation.h"
#include "Emu/Utility/System/Syscall/SyscallConvIF.h"
#include "Emu/Utility/System/ProcessState.h"
#include "Emu/Utility/System/Memory/MemorySystem.h"


namespace Onikiri {
    namespace RISCV64Linux {
        namespace Operation {

            using namespace EmulatorUtility;
            using namespace EmulatorUtility::Operation;

            typedef u64 RISCV64RegisterType;

            static const RISCV64RegisterType REG_VALUE_TRUE = 1;
            static const RISCV64RegisterType REG_VALUE_FALSE = 0;

            // Operands
            template <int OperandIndex>
            class RISCV64DstOperand
            {
            public:
                typedef RISCV64RegisterType type;
                static void SetOperand(OpEmulationState* opState, RISCV64RegisterType value)
                {
                    opState->SetDst(OperandIndex, value);
                }
            };

            template <int OperandIndex>
            struct RISCV64SrcOperand : public std::unary_function<OpEmulationState, RISCV64RegisterType>
            {
                typedef RISCV64RegisterType type;
                RISCV64RegisterType operator()(OpEmulationState* opState)
                {
                    return (RISCV64RegisterType)opState->GetSrc(OperandIndex);
                }
            };

            template <typename Type, Type value>
            struct RISCV64IntConst : public std::unary_function<OpEmulationState*, Type>
            {
                Type operator()(OpEmulationState* opState)
                {
                    return value;
                }
            };


            // interface
            inline void RISCV64DoBranch(OpEmulationState* opState, u64 target)
            {
                opState->SetTaken(true);
                opState->SetTakenPC(target);
            }

            inline u64 RISCV64NextPC(OpEmulationState* opState)
            {
                return (u64)opState->GetPC() + 4;
            }

            inline u64 RISCV64CurPC(OpEmulationState* opState)
            {
                return (u64)opState->GetPC();
            }


            // Set PC
            template<typename TSrc1>
            struct RISCV64Auipc : public std::unary_function<OpEmulationState, RegisterType>
            {
                RISCV64RegisterType operator()(OpEmulationState* opState) const
                {
                    return (TSrc1()(opState) << 12) + RISCV64CurPC(opState);
                }
            };

            // Load upper immediate
            template<typename TSrc1>
            struct RISCV64Lui : public std::unary_function<OpEmulationState, RegisterType>
            {
                RISCV64RegisterType operator()(OpEmulationState* opState) const
                {
                    return TSrc1()(opState) << 12;
                }
            };


            // compare
            template <typename TSrc1, typename TSrc2, typename Comp>
            struct RISCV64Compare : public std::unary_function<EmulatorUtility::OpEmulationState*, RISCV64RegisterType>
            {
                RISCV64RegisterType operator()(EmulatorUtility::OpEmulationState* opState)
                {
                    if (Comp()(TSrc1()(opState), TSrc2()(opState))) {
                        return (RISCV64RegisterType)REG_VALUE_TRUE;
                    }
                    else {
                        return (RISCV64RegisterType)REG_VALUE_FALSE;
                    }
                }
            };


            template <typename Type, typename TSrc1, typename TSrc2>
            struct RISCV64MIN : public std::unary_function<EmulatorUtility::OpEmulationState*, Type>
            {
                Type operator()(OpEmulationState* opState)
                {
                    Type lhs = TSrc1()(opState);
                    Type rhs = TSrc2()(opState);

                    if (lhs > rhs) {
                        return rhs;
                    }
                    else {
                        return lhs;
                    }
                }
            };


            template <typename Type, typename TSrc1, typename TSrc2>
            struct RISCV64MAX : public std::unary_function<EmulatorUtility::OpEmulationState*, Type>
            {
                Type operator()(OpEmulationState* opState)
                {
                    Type lhs = TSrc1()(opState);
                    Type rhs = TSrc2()(opState);

                    if (lhs < rhs) {
                        return rhs;
                    }
                    else {
                        return lhs;
                    }
                }
            };
            
            //
            // FP operations
            //

            template<typename TSrc>
            struct RISCV64NanBoxing : public std::unary_function<EmulatorUtility::OpEmulationState, u64>
            {
                u64 operator()(EmulatorUtility::OpEmulationState* opState) const
                {
                    return 0xffffffff00000000 | AsIntFunc<u32, f32>(TSrc()(opState));
                }
            };

            struct RISCV64RoundModeFromFCSR : public std::unary_function<EmulatorUtility::OpEmulationState, u64>
            {
                int operator()(EmulatorUtility::OpEmulationState* opState) const
                {
                    // TODO: Select a round mode from FCSR
                    return FE_TONEAREST;
                }
            };

            template <typename Type, typename TSrc1, typename TSrc2, typename TSrc3, typename RoundMode = IntConst<int, FE_ROUNDDEFAULT> >
            struct RISCV64MADD : public std::unary_function<OpEmulationState*, Type>
            {
                Type operator()(OpEmulationState* opState)
                {
                    Type lhs = TSrc1()(opState);
                    Type rhs = TSrc2()(opState);
                    Type ths = TSrc3()(opState); //変数名適当です

                    Onikiri::ScopedFESetRound sr(RoundMode()(opState));
                    volatile Type rvalue = std::fma(lhs, rhs, ths);
                    return rvalue;
                }
            };

            template <typename Type, typename TSrc1, typename TSrc2, typename TSrc3, typename RoundMode = IntConst<int, FE_ROUNDDEFAULT> >
            struct RISCV64MSUB : public std::unary_function<OpEmulationState*, Type>
            {
                Type operator()(OpEmulationState* opState)
                {
                    Type lhs = TSrc1()(opState);
                    Type rhs = TSrc2()(opState);
                    Type ths = TSrc3()(opState); //変数名適当です

                    Onikiri::ScopedFESetRound sr(RoundMode()(opState));
                    volatile Type rvalue = std::fma(lhs, rhs, -ths);
                    return rvalue;
                }
            };

            template <typename Type, typename TSrc1, typename TSrc2, typename TSrc3, typename RoundMode = IntConst<int, FE_ROUNDDEFAULT> >
            struct RISCV64NMADD : public std::unary_function<OpEmulationState*, Type>
            {
                Type operator()(OpEmulationState* opState)
                {
                    Type lhs = TSrc1()(opState);
                    Type rhs = TSrc2()(opState);
                    Type ths = TSrc3()(opState); //変数名適当です

                    Onikiri::ScopedFESetRound sr(RoundMode()(opState));
                    volatile Type rvalue = std::fma(-lhs, rhs, ths);
                    return rvalue;
                }
            };

            template <typename Type, typename TSrc1, typename TSrc2, typename TSrc3, typename RoundMode = IntConst<int, FE_ROUNDDEFAULT> >
            struct RISCV64NMSUB : public std::unary_function<OpEmulationState*, Type>
            {
                Type operator()(OpEmulationState* opState)
                {
                    Type lhs = TSrc1()(opState);
                    Type rhs = TSrc2()(opState);
                    Type ths = TSrc3()(opState); //変数名適当です

                    Onikiri::ScopedFESetRound sr(RoundMode()(opState));
                    volatile Type rvalue = std::fma(-lhs, rhs, -ths);
                    return rvalue;
                }
            };

            //conversion

            // TSrc の浮動小数点数をTypeの符号付き整数型に変換する．
            // Typeで表せる最大値を超えている場合は最大値を，最小値を下回っている場合は最小値を返す．
            template <typename Type, typename TSrc, typename RoundMode = IntConst<int, FE_ROUNDDEFAULT> >
            struct RISCV64FPToInt : public std::unary_function<EmulatorUtility::OpEmulationState*, Type>
            {
                Type operator()(EmulatorUtility::OpEmulationState* opState)
                {
                    typedef typename EmulatorUtility::signed_type<Type>::type SignedType;
                    typedef typename TSrc::result_type FPType;
                    // 2の補数を仮定
                    const SignedType maxValue = std::numeric_limits<SignedType>::max(); //ここ理解できてない（稲岡）
                    const SignedType minValue = std::numeric_limits<SignedType>::min();
                    FPType value = static_cast<FPType>(TSrc()(opState));

                    if (value > static_cast<FPType>(maxValue))
                        return maxValue;
                    else if (value < static_cast<FPType>(minValue))
                        return minValue;
                    else
                        return static_cast<Type>(value);
                }
            };

            //符号なし整数型に変換
            template <typename Type, typename TSrc, typename RoundMode = IntConst<int, FE_ROUNDDEFAULT> >
            struct RISCV64FPToIntU : public std::unary_function<EmulatorUtility::OpEmulationState*, Type>
            {
                Type operator()(EmulatorUtility::OpEmulationState* opState)
                {
                    typedef typename EmulatorUtility::unsigned_type<Type>::type UnsignedType;
                    typedef typename TSrc::result_type FPType;
                    // 2の補数を仮定
                    const UnsignedType maxValue = std::numeric_limits<UnsignedType>::max();
                    const UnsignedType minValue = std::numeric_limits<UnsignedType>::min();
                    FPType value = static_cast<FPType>(TSrc()(opState));

                    if (value > static_cast<FPType>(maxValue))
                        return maxValue;
                    else if (value < static_cast<FPType>(minValue))
                        return minValue;
                    else
                        return static_cast<Type>(value);
                }
            };

            // sign from src1 and the rest from src2
            //FLOAT版、GenericOperationに入れてもいいかも
            template <typename TSrc1, typename TSrc2>
            struct FPFLOATCopySign : public std::unary_function<OpEmulationState*, u64>
            {
                u32 operator()(OpEmulationState* opState)
                {
                    u32 lhs = static_cast<u32>(TSrc1()(opState));
                    u32 rhs = static_cast<u32>(TSrc2()(opState));

                    return (lhs & (u32)0x80000000) | (rhs & ~(u32)0x80000000);
                }
            };


            template <typename TSrc1, typename TSrc2>
            struct FPFLOATCopySignNeg : public std::unary_function<OpEmulationState*, u64>
            {
                u32 operator()(OpEmulationState* opState)
                {
                    u32 lhs = static_cast<u32>(TSrc1()(opState));
                    u32 rhs = static_cast<u32>(TSrc2()(opState));

                    return (~lhs & (u32)0x80000000) | (rhs & ~(u32)0x80000000);
                }
            };

            template <typename TSrc1, typename TSrc2>
            struct FPDoubleCopySignXor : public std::unary_function<OpEmulationState*, u64>
            {
                u64 operator()(OpEmulationState* opState)
                {
                    u64 lhs = static_cast<u64>(TSrc1()(opState));
                    u64 rhs = static_cast<u64>(TSrc2()(opState));

                    return (lhs & (u64)0x8000000000000000ULL) ^ rhs;
                }
            };


            template <typename TSrc1, typename TSrc2>
            struct FPFLOATCopySignXor : public std::unary_function<OpEmulationState*, u64>
            {
                u32 operator()(OpEmulationState* opState)
                {
                    u32 lhs = static_cast<u32>(TSrc1()(opState));
                    u32 rhs = static_cast<u32>(TSrc2()(opState));

                    return (lhs & (u32)0x80000000) ^ rhs;
                }
            };

            //compare

            template <typename Type, typename TSrc1, typename TSrc2>
            struct RISCV64FPEqual : public std::unary_function<EmulatorUtility::OpEmulationState*, u64> //あってる？
            {
                bool operator()(EmulatorUtility::OpEmulationState* opState)
                {
                    Type lhs = static_cast<Type>(TSrc1()(opState));
                    Type rhs = static_cast<Type>(TSrc2()(opState));

                    return lhs == rhs;
                }
            };

            template <typename Type, typename TSrc1, typename TSrc2>
            struct RISCV64FPLess : public std::unary_function<EmulatorUtility::OpEmulationState*, u64> //あってる？
            {
                bool operator()(EmulatorUtility::OpEmulationState* opState)
                {
                    Type lhs = static_cast<Type>(TSrc1()(opState));
                    Type rhs = static_cast<Type>(TSrc2()(opState));

                    return lhs < rhs;
                }
            };

            template <typename Type, typename TSrc1, typename TSrc2>
            struct RISCV64FPLessEqual : public std::unary_function<EmulatorUtility::OpEmulationState*, u64> //あってる？
            {
                bool operator()(EmulatorUtility::OpEmulationState* opState)
                {
                    Type lhs = static_cast<Type>(TSrc1()(opState));
                    Type rhs = static_cast<Type>(TSrc2()(opState));

                    return lhs <= rhs;
                }
            };

            //ここを頑張る
            template <typename Type, typename TSrc1>
            struct RISCV64FCLASS : public std::unary_function<EmulatorUtility::OpEmulationState*, u64>
            {
                u64 operator()(EmulatorUtility::OpEmulationState* opState)
                {
                    Type value = static_cast<Type>(TSrc1()(opState));
                    switch(std::fpclassify(value)) {
                case FP_INFINITE:
                            if(std::signbit(value)) return (u64)0x0000000000000001;
                            else return (u64)0x0000000000000080;
                            break;
                case FP_NAN:       return (u64)0x0000000000000100;
                case FP_NORMAL:
                            if(std::signbit(value)) return (u64)0x0000000000000002;
                            else return (u64)0x0000000000000040;
                            break;
                case FP_SUBNORMAL:
                            if(std::signbit(value)) return (u64)0x0000000000000004;
                            else return (u64)0x0000000000000020;
                            break;
                case FP_ZERO:
                            if(std::signbit(value)) return (u64)0x0000000000000008;
                            else return (u64)0x0000000000000010;
                            break;
                default:           return 0x0000000000000000;
                }
                }
            };


            // Branch
            template <typename TSrcTarget, typename TSrcDisp>
            inline void RISCV64BranchAbsUncond(OpEmulationState* opState)
            {
                u64 target = TSrcTarget()(opState) + cast_to_signed(TSrcDisp()(opState));
                RISCV64DoBranch(opState, target);
            }

            template <typename TSrcDisp>
            inline void RISCV64BranchRelUncond(OpEmulationState* opState)
            {
                u64 target = RISCV64CurPC(opState) + cast_to_signed(TSrcDisp()(opState));
                RISCV64DoBranch(opState, target);
            }

            template <typename TSrcDisp, typename TCond>
            inline void RISCV64BranchRelCond(OpEmulationState* opState)
            {
                if (TCond()(opState)) {
                    RISCV64BranchRelUncond<TSrcDisp>(opState);
                }
                else {
                    opState->SetTakenPC(RISCV64NextPC(opState));
                }
            }


            template <typename TDest, typename TSrcDisp>
            inline void RISCV64CallRelUncond(OpEmulationState* opState)
            {
                RISCV64RegisterType ret = static_cast<RISCV64RegisterType>(RISCV64NextPC(opState));
                RISCV64BranchRelUncond<TSrcDisp>(opState);
                TDest::SetOperand(opState, ret);
            }

            template <typename TDest, typename TSrcTarget, typename TSrcDisp>
            inline void RISCV64CallAbsUncond(OpEmulationState* opState)
            {
                RISCV64RegisterType ret = static_cast<RISCV64RegisterType>(RISCV64NextPC(opState));
                RISCV64BranchAbsUncond<TSrcTarget, TSrcDisp>(opState);
                TDest::SetOperand(opState, ret);
            }

            //
            // Load/Store
            //
            template<typename TSrc1, typename TSrc2>
            struct RISCV64Addr : public std::unary_function<EmulatorUtility::OpEmulationState, RISCV64RegisterType>
            {
                RISCV64RegisterType operator()(EmulatorUtility::OpEmulationState* opState) const
                {
                    return TSrc1()(opState) + EmulatorUtility::cast_to_signed(TSrc2()(opState));
                }
            };

            //
            // Div/Rem
            //
            template <typename TSrc1, typename TSrc2>
            struct RISCV64IntDiv : public std::unary_function<OpEmulationState*, u64>
            {
                u64 operator()(OpEmulationState* opState)
                {
                    s64 src1 = static_cast<s64>(TSrc1()(opState));
                    s64 src2 = static_cast<s64>(TSrc2()(opState));
                    if (src2 == 0) {
                        return static_cast<u64>(-1);
                    }
                    if (src1 < -0x7fffffffffffffff && src2 == -1) { // overflow
                        return src1;
                    }
                    return static_cast<u64>(src1 / src2);
                }
            };

            template <typename TSrc1, typename TSrc2>
            struct RISCV64IntRem : public std::unary_function<OpEmulationState*, u64>
            {
                u64 operator()(OpEmulationState* opState)
                {
                    s64 src1 = static_cast<s64>(TSrc1()(opState));
                    s64 src2 = static_cast<s64>(TSrc2()(opState));
                    if (src2 == 0) {
                        return src1;
                    }
                    if (src1 < -0x7fffffffffffffff && src2 == -1) { // overflow
                        return 0;
                    }
                    return src1 % src2;
                }
            };

            template <typename TSrc1, typename TSrc2>
            struct RISCV64IntDivu : public std::unary_function<OpEmulationState*, u64>
            {
                u64 operator()(OpEmulationState* opState)
                {
                    u64 src1 = static_cast<u64>(TSrc1()(opState));
                    u64 src2 = static_cast<u64>(TSrc2()(opState));
                    if (src2 == 0) {
                        return static_cast<u64>(-1);
                    }
                    return src1 / src2;
                }
            };

            template <typename TSrc1, typename TSrc2>
            struct RISCV64IntRemu : public std::unary_function<OpEmulationState*, u64>
            {
                u64 operator()(OpEmulationState* opState)
                {
                    u64 src1 = static_cast<u64>(TSrc1()(opState));
                    u64 src2 = static_cast<u64>(TSrc2()(opState));
                    if (src2 == 0) {
                        return src1;
                    }
                    return src1 % src2;
                }
            };

            template <typename TSrc1, typename TSrc2>
            struct RISCV64IntDivw : public std::unary_function<OpEmulationState*, u32>
            {
                u32 operator()(OpEmulationState* opState)
                {
                    s32 src1 = static_cast<s32>(TSrc1()(opState));
                    s32 src2 = static_cast<s32>(TSrc2()(opState));
                    if (src2 == 0) {
                        return static_cast<u32>(-1);
                    }
                    if (src1 < -0x7fffffff && src2 == -1) { // overflow
                        return src1;
                    }
                    return static_cast<u32>(src1 / src2);
                }
            };

            template <typename TSrc1, typename TSrc2>
            struct RISCV64IntRemw : public std::unary_function<OpEmulationState*, u32>
            {
                u32 operator()(OpEmulationState* opState)
                {
                    s32 src1 = static_cast<s32>(TSrc1()(opState));
                    s32 src2 = static_cast<s32>(TSrc2()(opState));
                    if (src2 == 0) {
                        return src1;
                    }
                    if (src1 < -0x7fffffff && src2 == -1) { // overflow
                        return 0;
                    }
                    return src1 % src2;
                }
            };

            template <typename TSrc1, typename TSrc2>
            struct RISCV64IntDivuw : public std::unary_function<OpEmulationState*, u32>
            {
                u32 operator()(OpEmulationState* opState)
                {
                    u32 src1 = static_cast<u32>(TSrc1()(opState));
                    u32 src2 = static_cast<u32>(TSrc2()(opState));
                    if (src2 == 0) {
                        return static_cast<u32>(-1);
                    }
                    return src1 / src2;
                }
            };

            template <typename TSrc1, typename TSrc2>
            struct RISCV64IntRemuw : public std::unary_function<OpEmulationState*, u32>
            {
                u32 operator()(OpEmulationState* opState)
                {
                    u32 src1 = static_cast<u32>(TSrc1()(opState));
                    u32 src2 = static_cast<u32>(TSrc2()(opState));
                    if (src2 == 0) {
                        return src1;
                    }
                    return src1 % src2;
                }
            };

            template <typename Type, typename TSrc1, typename TSrc2>
            struct RISCV64Min : public std::unary_function<OpEmulationState*, u64>
            {
                u64 operator()(OpEmulationState* opState)
                {
                    Type src1 = static_cast<Type>(TSrc1()(opState));
                    Type src2 = static_cast<Type>(TSrc2()(opState));
                    return static_cast<u64>(std::min(src1, src2));
                }
            };

            template <typename Type, typename TSrc1, typename TSrc2>
            struct RISCV64Max : public std::unary_function<OpEmulationState*, u64>
            {
                u64 operator()(OpEmulationState* opState)
                {
                    Type src1 = static_cast<Type>(TSrc1()(opState));
                    Type src2 = static_cast<Type>(TSrc2()(opState));
                    return static_cast<u64>(std::max(src1, src2));
                }
            };

            template <typename Type, typename TAddr>
            struct RISCV64AtomicLoad : public std::unary_function<OpEmulationState*, u64>
            {
                u64 operator()(OpEmulationState* opState)
                {
                    MemorySystem* mem = opState->GetProcessState()->GetMemorySystem();
                    MemAccess access;
                    access.address.pid = opState->GetPID();
                    access.address.address = TAddr()(opState);
                    access.size = sizeof(Type);
                    access.sign = true;

                    mem->ReadMemory(&access);

                    if (access.result != MemAccess::Result::MAR_SUCCESS) {
                        THROW_RUNTIME_ERROR("Atomic memory read access failed: %s", access.ToString().c_str());
                    }

                    return access.value;
                }
            };

            template <typename Type, typename TValue, typename TAddr>
            void RISCV64AtomicStore(OpEmulationState* opState)
            {
                MemorySystem* mem = opState->GetProcessState()->GetMemorySystem();
                MemAccess access;
                access.address.pid = opState->GetPID();
                access.address.address = TAddr()(opState);
                access.size = sizeof(Type);
                access.value = TValue()(opState);

                mem->WriteMemory(&access);

                if (access.result != MemAccess::Result::MAR_SUCCESS) {
                    THROW_RUNTIME_ERROR("Atomic memory write access failed: %s", access.ToString().c_str());
                }
            }

            template <typename Type, typename TDst, typename TAddr, typename Operation>
            void RISCV64AtomicOperation(OpEmulationState* opState)
            {
                Set<TDst, RISCV64AtomicLoad<Type, TAddr>>(opState);
                RISCV64AtomicStore<Type, Operation, TAddr>(opState);
            }

            inline void RISCV64SyscallSetArg(EmulatorUtility::OpEmulationState* opState)
            {
                EmulatorUtility::SyscallConvIF* syscallConv = opState->GetProcessState()->GetSyscallConv();
                syscallConv->SetArg(0, SrcOperand<0>()(opState));
                syscallConv->SetArg(1, SrcOperand<1>()(opState));
                syscallConv->SetArg(2, SrcOperand<2>()(opState));
                DstOperand<0>::SetOperand(opState, SrcOperand<0>()(opState));
            }

            // invoke syscall, get result&error and branch if any
            inline void RISCV64SyscallCore(EmulatorUtility::OpEmulationState* opState)
            {
                EmulatorUtility::SyscallConvIF* syscallConv = opState->GetProcessState()->GetSyscallConv();
                syscallConv->SetArg(3, SrcOperand<1>()(opState));
                syscallConv->SetArg(4, SrcOperand<2>()(opState));
                //syscallConv->SetArg(5, SrcOperand<2>()(opState));
                syscallConv->Execute(opState);

                u64 error = (u64)syscallConv->GetResult(EmulatorUtility::SyscallConvIF::ErrorFlagIndex);
                u64 val = (u64)syscallConv->GetResult(EmulatorUtility::SyscallConvIF::RetValueIndex);
                DstOperand<0>::SetOperand(opState, error ? (u64)-1 : val);
            }

            template <typename TDest, typename CSR_D, typename TSrc1, typename CSR_S>
            inline void RISCV64CSRRW(OpEmulationState* opState)
            {
                TDest::SetOperand(opState, 0);
                RUNTIME_WARNING("CSR is accessed.");
                //TDest::SetOperand(opState, CSR_S()(opState));
                //CSR_D::SetOperand(opState, TSrc1()(opState));
            }

            template <typename TDest, typename CSR_D, typename TSrc1, typename CSR_S>
            inline void RISCV64CSRRS(OpEmulationState* opState)
            {
                TDest::SetOperand(opState, 0);
                RUNTIME_WARNING("CSR is accessed.");
                //TDest::SetOperand(opState, CSR_S()(opState));
                //CSR_D::SetOperand(opState, TSrc1()(opState) | CSR_S()(opState));
            }

            template <typename TDest, typename CSR_D, typename TSrc1, typename CSR_S>
            inline void RISCV64CSRRC(OpEmulationState* opState)
            {
                TDest::SetOperand(opState, 0);
                RUNTIME_WARNING("CSR is accessed.");
                //TDest::SetOperand(opState, CSR_S()(opState));
                //CSR_D::SetOperand(opState, ~(TSrc1()(opState)) & CSR_S()(opState));
            }
            

        } // namespace Operation {
    } // namespace RISCV64Linux {
} // namespace Onikiri

#endif // #ifndef EMU_ALPHA_LINUX_ALPHALINUX_ALPHAOPERATION_H
