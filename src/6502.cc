#include <cstdint>
#include <cinttypes>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <string>
#include <chrono>

#include <unistd.h> // for usleep
//#include <ctime>    // for dt

// TODO: suffix with CLR?
#define LABEL   "\e[0;1;33m"
#define BORDER  "\e[0;1m"
#define DIM_CLR "\e[0;90m"
#define BRT_CLR "\e[0;97m"

using u8  = std::uint8_t;
using u16 = std::uint16_t;

using u64 = std::uint64_t;

using i8  = std::int8_t;
using i16 = std::int16_t;

using f32 = float;

// TODO: memory display modes? HEX/ASCII
// TODO: keyboard input
// TODO: only update changes in widgets instead of reprinting them
// TODO: colour code memory values based on access frequency and temporal proximity
// TODO: memcopy command-line arg ROM file into system RAM
// TODO: do more testing	
// TODO: handle under/overflow in stack?

[[nodiscard]] inline auto
timer() noexcept -> f32
{
	using clock_t = std::chrono::high_resolution_clock;
	static clock_t::time_point            start   = clock_t::now();
	std::chrono::duration<f32,std::milli> elapsed = clock_t::now() - start;
	return elapsed.count();
};

struct keyboard_t {
	// TODO: implement!
	// TODO: handle meta keys
	// TODO: handle up/down/pressed?
	void
	update() noexcept
	{
		assert( port );
		u8 keycode = 0; // TODO: read key press
		*port      = keycode;
	}
	u8 *port = nullptr;
};

// TODO: clean these two temporary hacks up
inline auto
init_hdr( char *hdr, u8 const width ) noexcept
{
	auto hack = "───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────";
	std::snprintf( hdr, 1024, "╭%.*s╮", 3*width, hack );
	return true;
}

inline auto
init_ftr( char *ftr, u8 const width ) noexcept
{
	auto hack = "───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────";
	std::snprintf( ftr, 1024, "╰%.*s╯", 3*width, hack );
	return true;
}

inline auto
init_rst( char *rst, u8 const width, u8 const height )
{
	std::snprintf( rst, 3, "\033[H" );
	return true;
}

template <u8 width, u8 height>
struct terminal_display_t {
	void
	update() noexcept
	{
		static char hdr[3*(width+2)+1],
		            ftr[3*(width+2)+1];
		
		static bool all_initialized = {
			init_hdr(hdr,width) and
			init_ftr(ftr,width)
		}; (void) all_initialized;
		
		constexpr u16 last_row_addr = (height-1) * width;
		
		assert( block_start );
		std::printf( "\033[?25l\033[%u;%uH" "%s" "\033[B\033[%uD", y, x, hdr, width+2 ); 
		for ( u16 row_addr_offset  = 0;
		          row_addr_offset  < last_row_addr;
		          row_addr_offset += width )
		{
			std::printf( "│%-*s│" "\033[B\033[%uD", width, block_start + row_addr_offset, width+2 );
		}
		std::puts( ftr );
	}
	
	u8 const *block_start = nullptr;
	u8 x=0, y=0;
};

namespace cpu {
	enum cpu_flag_e: u8 {
		C  = 0, // Carry
		Z  = 1, // Zero
		I  = 2, // Interrupt disable
		D  = 3, // Decimal
		BL = 4, // B-flag, lower byte
		BU = 5, // B-flag, upper byte
		V  = 6, // oVerflow
		N  = 7  // Negative
	};

	struct cpu_t {
		u16 PC = 0; //     program counter
		u8  A  = 0, // accumulator register
			 X  = 0, //       index register
			 Y  = 0, //       index register
			 P  = 0, //      status register
			 SP = 0; //       stack pointer
		
		u16 Hz = 3000; // clock speed
		
		// Just used for meta insight
		u64 cc = 0;// cycle count
		std::string last_op = "";
		
		inline void
		clear_flag( cpu_flag_e const f ) noexcept
		{
			assert( f < 8 );    
			P &= ~(1U << f);
		}
		
		inline void
		set_flag( cpu_flag_e const f ) noexcept
		{
			assert( f < 8 );
			P |= (1U << f);
		}
		
		inline void
		flip_flag( cpu_flag_e const f ) noexcept
		{
			assert( f < 8 );
			P ^= (1U << f);
		}
		
		inline void
		set_flag( cpu_flag_e const f, bool const value ) noexcept
		{
			assert( f < 8 );
			if ( value )
				set_flag( f );
			else
				clear_flag( f );
		}
		
		inline auto
		read_flag( cpu_flag_e const f ) const noexcept -> bool
		{
			return (P >> f) & 1U;
		}
			
		inline void
		update_ZN( u8 &byte ) noexcept
		{
			set_flag( Z, byte==0 ); // set zero flag if result is 0
			set_flag( N, byte>>7 ); // set sign flag to MSD sign bit
		}
		
		inline void
		update_ZN() noexcept
		{
			update_ZN( A );
		}
	};
} // namespace cpu end

struct system_t {
	cpu::cpu_t CPU; // MOS Technology 6502
	
	//u8 RAM[ram_kb];
	u8 RAM[0xFFFF+1];
	
	inline void
	push( u8 const byte ) noexcept
	{
		// TODO: optional overflow guard? wrap?
		RAM[0x0100 + CPU.SP++] = byte; // TODO: verify
	}
	
	inline void
	push_addr( u16 const addr ) noexcept
	{
		// TODO: optional overflow guard? wrap?
		RAM[0x0100 + CPU.SP++] = addr >> 8;
		RAM[0x0100 + CPU.SP++] = addr  & 0xFF;
	}
	
	inline auto
	pull() noexcept -> u8
	{
		// TODO: optional underflow guard? wrap?
		return RAM[0x0100 + CPU.SP--]; // TODO: verify
	}
	
	inline auto
	pull_addr() noexcept -> u16
	{
		// TODO: optional underflow guard? wrap?
		CPU.SP -= 2;
		return (static_cast<u16>(RAM[0x0100 + CPU.SP]) << 8) + RAM[0x0100 + CPU.SP+1];
	}
	
	// Relative Jump (returns true if page border was crossed)
	inline auto
	rel_jmp() noexcept -> bool
	{
		auto tmp = CPU.PC >> 8;                        // store previous page number in a buffer
		CPU.PC += reinterpret_cast<i8&>( read_imm() ); // do the relative jump
		return tmp != (CPU.PC >> 8);                   // return true if the jump crossed a page border
	}
	
	// Bitwise OR (of accumulator A)
	inline void
	ORA( u8 const byte ) noexcept
	{
		CPU.A |= byte;   // perform bitwise OR on accumulator A
		CPU.update_ZN(); // update zero and sign flags
	}
	
	// Bitwise XOR (of accumulator A)
	inline void
	EOR( u8 const byte ) noexcept
	{
		CPU.A ^= byte;   // perform bitwise XOR on accumulator A
		CPU.update_ZN(); // update zero and sign flags
	}
	
	// Bitwise AND (of accumulator A)
	inline void
	AND( u8 const byte ) noexcept
	{
		CPU.A &= byte;   // perform bitwise AND on accumulator A
		CPU.update_ZN(); // update zero and sign flags
	}
	
	// Rotate Left
	inline void
	ROL( u8 &byte ) noexcept
	{
		using namespace cpu;
		auto C2 = CPU.read_flag(C); // buffer old carry flag
		CPU.set_flag( C, byte>>7 ); // set carry flag to MSD
		byte <<= 1;                 // shift byte left by 1
		byte  += C2;                // set LSD to old carry flag
		CPU.update_ZN(byte);        // update zero and sign flags
	}	
	
	// Rotate Right
	inline void
	ROR( u8 &byte ) noexcept
	{
		using namespace cpu;
		auto C2 = CPU.read_flag(C); // buffer old carry flag
		CPU.set_flag( C, byte&1 );  // set carry flag to LSD
		byte >>= 1;                 // shift byte right by 1
		byte  += C2 << 7;           // set MSD to old carry flag
		CPU.update_ZN(byte);        // update zero and sign flags
	}
	
	// Arithmetic Shift Left
	inline void
	ASL( u8 &byte ) noexcept
	{
		using namespace cpu;
		CPU.set_flag( C, byte>>7 ); // set carry flag to MSD
		byte <<= 1;                 // shift byte left by 1
		CPU.update_ZN(byte);        // update zero and sign flags
	}
	
	// Logical Shift Right
	inline void
	LSR( u8 &byte ) noexcept
	{
		using namespace cpu;
		CPU.set_flag( C, byte&1 ); // set carry flag to LSD
		byte >>= 1;                // shift byte right by 1
		CPU.update_ZN(byte);       // update zero and sign flags
	}
	
	// Subtract with carry (from accumulator A)
	inline void
	SBC( u8 const byte ) noexcept
	{ // TODO: handle decimal case
		using namespace cpu;
		u16 tmp  = CPU.A - !(CPU.read_flag(C)) + byte;             // subtract from a buffer (u16 to catch overflows)
		CPU.A    = tmp  & 0xFF;                                    // update accumulator A with the least significant byte
		CPU.set_flag( C, tmp<=0xFF );                              // clear the carry flag if the subtraction overflows
		CPU.set_flag( V, ~(CPU.A ^ byte) & (CPU.A ^ tmp) & 0x80 ); // TODO: describe
		CPU.update_ZN();                                           // update zero and sign flags
	}
	
	// Add with carry (to accumulator A)
	inline void
	ADC( u8 const byte ) noexcept
	{
		using namespace cpu;
		if ( CPU.read_flag(D) ) { // BCD mode
			u8  tmp_l = CPU.A&0x0F + byte&0x0F;
			if ( tmp_l > 0x09 )
				tmp_l += 0x06;
			u16 tmp = CPU.A&0xF0 + byte&0xF0 + tmp_l;
			CPU.set_flag( C, tmp > 0x99 );
			CPU.A = tmp & 0xFF;
		}
		else { // Binary mode
			u16 tmp = CPU.A + CPU.read_flag(C) + byte;                 // add to a buffer (u16 to catch overflows)
			CPU.A   = tmp & 0xFF;                                      // update accumulator A with the least significant byte
			CPU.set_flag( C, tmp > 0xFF );                             // update carry flag if the most significant byte >0
			CPU.set_flag( V, ~(CPU.A ^ byte) & (CPU.A ^ tmp) & 0x80 ); // TODO: describe
		}
		CPU.update_ZN(); // update zero and sign flags
	}
		
	// Increment memory
	inline void
	INC( u8 &byte ) noexcept
	{
		++byte;
		CPU.update_ZN(byte);
	}
	
	// Decrement memory
	inline void
	DEC( u8 &byte ) noexcept
	{
		--byte;
		CPU.update_ZN(byte);
	}
	
	inline void
	compare( u8 const x, u8 const mem ) noexcept
	{
		using namespace cpu;
		u8 tmp  = x - mem;
		CPU.set_flag( C, 0 ); // TODO
		CPU.update_ZN(tmp);
	}
	
	[[nodiscard]] inline auto
	read_addr() noexcept -> u16
	{
		CPU.PC += 2;
		return (static_cast<u16>(RAM[CPU.PC-1]) << 8) + RAM[CPU.PC-2];
	}
	
	[[nodiscard]] inline auto
	read( u16 const addr ) noexcept -> u8&
	{
		return RAM[addr];
	}
	
	[[nodiscard]] inline auto
	read_imm() noexcept -> u8&
	{
		return read( CPU.PC++ );
	}
	
	[[nodiscard]] inline auto
	read_abs() noexcept -> u8&
	{
		return read( read_addr() );
	}
	
	[[nodiscard]] inline auto
	read_abs_x( u8 &cycles ) noexcept -> u8&
	{
		// TODO: refactor
		u16 const base_addr = read_addr();
		u16 const base_page = base_addr & 0xFF00;
		u16 const offs_addr = base_addr + CPU.X;
		u16 const offs_page = offs_addr & 0xFF00;
		if ( base_page != offs_page )
			++cycles;
		return read( offs_addr );
	}
	
	[[nodiscard]] inline auto
	read_abs_y( u8 &cycles ) noexcept -> u8&
	{
		// TODO: refactor
		u16 const base_addr = read_addr();
		u16 const base_page = base_addr & 0xFF00;
		u16 const offs_addr = base_addr + CPU.Y;
		u16 const offs_page = offs_addr & 0xFF00;
		if ( base_page != offs_page )
			++cycles;
		return read( offs_addr );
	}
	
	[[nodiscard]] inline auto
	read_zpg( u8 const offset=0 ) noexcept -> u8&
	{
		return read( (RAM[CPU.PC++] + offset) & 0xFF );	
	}
		
	[[nodiscard]] inline auto
	read_zpg_x() noexcept -> u8&
	{
		return read_zpg( CPU.X );
	}
	
	[[nodiscard]] inline auto
	read_zpg_y() noexcept -> u8&
	{
		return read_zpg( CPU.Y );
	}
	
	[[nodiscard]] inline auto
	read_ind_x() noexcept -> u8&
	{
		return read( read( (read_imm() + CPU.X) & 0xFF ) );
	}
	
	[[nodiscard]] inline auto
	read_ind_y( u8 &cycles ) noexcept -> u8&
	{
		u16 const base_addr = read( read_imm() );
		u16 const base_page = base_addr & 0xFF00;
		u16 const offs_addr = base_addr + CPU.Y;
		u16 const offs_page = offs_addr & 0xFF00;
		if ( base_page != offs_page )
			++cycles;
		return read( offs_addr );
	}
	
	void
	update() noexcept
	{
		using namespace cpu;
		static u8 cycles = 0;
		
		// TODO: fix cycle elapse pre OP side-effects instead of post
		if ( cycles ) {
			auto cycle_length_us = (int)(1000000.0f / CPU.Hz);
			usleep( cycle_length_us );
			--cycles;
			++CPU.cc;
			return;
		}
		
		u8 instruction = RAM[CPU.PC++];
		
		switch (instruction)
		{
			// BRK impl
			case 0x00: {
				CPU.last_op = "BRK impl";
				cycles      = 7;
				CPU.PC      = (RAM[0xFFFF] << 8) + RAM[0xFFFE]; // TODO: verify
				CPU.set_flag( I, 1 );
				push( CPU.PC );
				push( CPU.P  );
				while(1); // TODO: remove
			} break; 
			
			// ORA X,ind
			case 0x01: {
				CPU.last_op = "ORA X,ind";
				cycles      = 6;
				ORA( read_ind_x() );
			} break;
			
			// ORA zpg
			case 0x05: {
				CPU.last_op = "ORA zpg";
				cycles      = 3;
				ORA( read_zpg() );
			} break;
			
			// ASL zpg
			case 0x06: {
				CPU.last_op = "ASL zpg";
				cycles      = 5;
				ASL( read_zpg() );
			} break;
			
			// PHP impl
			case 0x08: {
				CPU.last_op = "PHP impl";
				cycles      = 3;
				push( CPU.P );
			} break;
			
			// ORA imm
			case 0x09: {
				CPU.last_op = "ORA imm";
				cycles      = 2;
				ORA( read_imm() );
			} break;
			
			// ASL acc
			case 0x0A: {
				CPU.last_op = "ASL acc";
				cycles      = 2;
				ASL( CPU.A );
			} break;
			
			// ORA abs
			case 0x0D: {
				CPU.last_op = "ORA abs";
				cycles      = 4;
				ORA( read_abs() );
			} break;
			
			// ASL abs
			case 0x0E: {
				CPU.last_op = "ASL abs";
				cycles      = 6;
				ASL( read_abs() );
			} break;
			
			// BPL rel
			case 0x10: {
				CPU.last_op = "BPL rel";
				cycles      = 2;
				if ( !CPU.read_flag(N) )
					cycles += rel_jmp()? 2 : 1; 
			} break;
			
			// ORA ind,Y
			case 0x11: {
				CPU.last_op = "ORA ind,Y";
				cycles      = 5;
				ORA( read_ind_y(cycles) );
			} break;
			
			// ORA zpg,X
			case 0x15: {
				CPU.last_op = "ORA zpg,X";
				cycles      = 4;
				ORA( read_zpg_x() );
			} break;
			
			// ASL zpg,X
			case 0x16: {
				CPU.last_op = "ASL zpg,X";
				cycles      = 6;
				ASL( read_zpg_x() );
			} break;
			
			// CLC impl
			case 0x18: {
				CPU.last_op = "CLC impl";
				cycles      = 2;
				CPU.clear_flag( C );
			} break;
			
			// ORA abs,Y
			case 0x19: {
				CPU.last_op = "ORA abs,Y";
				cycles      = 4;
				ORA( read_abs_y(cycles) );
			} break;
			
			// ORA abs,X
			case 0x1D: {
				CPU.last_op = "ORA abs,X";
				cycles      = 4;
				ORA( read_abs_x(cycles) );
			} break;
			
			// ASL abs,X
			case 0x1E: {
				CPU.last_op = "ASL abs,X";
				cycles      = 7;
				u8 dummy;
				ASL( read_abs_x(dummy) ); // TODO: clean up
			} break;
			
			// JSR abs
			case 0x20: {
				CPU.last_op = "JSR abs";
				cycles      = 6;
				push_addr( CPU.PC+2 );
				CPU.PC      = read_addr();
			} break;
			
			// AND ind,X
			case 0x21: {
				CPU.last_op = "AND ind,X";
				cycles      = 6;
				AND( read_ind_x() );
			} break;
			
			// BIT zpg
			case 0x24: {
				CPU.last_op = "BIT zpg";
				cycles      = 3;
				auto tmp    = CPU.A & read_zpg();
				CPU.set_flag( N, tmp>>7 );
				CPU.set_flag( V, (tmp>>6)&1 );
			} break;
			
			// AND zpg
			case 0x25: {
				CPU.last_op = "AND zpg";
				cycles      = 3;
				AND( read_zpg() );
			} break;
			
			// ROL zpg
			case 0x26: {
				CPU.last_op = "ROL zpg";
				cycles      = 5;
				ROL( read_zpg() );
			} break;
			
			// PLP impl
			case 0x28: {
				CPU.last_op = "PLP impl";
				// TODO: verify break restoration veracity
				cycles      = 4;
				CPU.P       = pull();
			} break;
			
			// AND imm
			case 0x29: {
				CPU.last_op = "AND imm";
				cycles      = 2;
				AND( read_imm() );
			} break;
			
			// ROL acc
			case 0x2A: {
				CPU.last_op = "ROL acc";
				cycles      = 2;
				ROL( CPU.A );
			} break;
			
			// BIT abs
			case 0x2C: {
				CPU.last_op = "BIT abs";
				
			} break;
			
			// AND abs
			case 0x2D: {
				CPU.last_op = "AND abs";
				cycles      = 4;
				AND( read_abs() );
			} break;
			
			// ROL abs
			case 0x2E: {
				CPU.last_op = "ROL abs";
				cycles      = 6;
				ROL( read_abs() );
			} break;
			
			// BMI rel
			case 0x30: {
				CPU.last_op = "BMI rel";
				cycles      = 2;
				if ( CPU.read_flag(N) )
					cycles += rel_jmp()? 2 : 1; 
			} break;
			
			// AND ind,Y
			case 0x31: {
				CPU.last_op = "AND ind,Y";
				cycles      = 5;
				AND( read_ind_y(cycles) );
			} break;
			
			// AND zpg,X
			case 0x35: {
				CPU.last_op = "AND zpg,X";
				cycles      = 4;
				AND( read_zpg_x() );
			} break;
			
			// ROL zpg,X
			case 0x36: {
				CPU.last_op = "ROL zpg,X";
				cycles      = 6;
				ROL( read_zpg_x() );
			} break;
			
			// SEC impl
			case 0x38: {
				CPU.last_op = "SEC impl";
				cycles      = 2;
				CPU.set_flag( C );
			} break;
			
			// AND abs,Y
			case 0x39: {
				CPU.last_op = "AND abs,Y";
				cycles      = 4;
				AND( read_abs_y(cycles) );
			} break;
			
			// AND abs,X
			case 0x3D: {
				CPU.last_op = "AND abs,X";
				cycles      = 4;
				AND( read_abs_x(cycles) );
			} break;
			
			// ROL abs,X
			case 0x3E: {
				CPU.last_op = "ROL abs,X";
				cycles      = 7;
				u8 dummy;
				ROL( read_abs_x(dummy) ); // TODO: clean up
			} break;
			
			// RTI impl
			case 0x40: { // TODO: verify endianness
				CPU.last_op = "RTI impl";
				cycles      = 6;
				CPU.P       = pull();
				CPU.PC      = pull(); // MSB
				CPU.PC    <<= 8;
				CPU.PC     += pull(); // LSB
			} break;
			
			// EOR ind,X
			case 0x41: {
				CPU.last_op = "EOR ind,X";
				cycles      = 6;
				EOR( read_ind_x() );
			} break;
			
			// EOR zpg
			case 0x45: {
				CPU.last_op = "EOR zpg";
				cycles      = 3;
				EOR( read_zpg() );
			} break;
			
			// LSR zpg
			case 0x46: {
				CPU.last_op = "LSR zpg";
				cycles      = 5;
				LSR( read_zpg() );
			} break;
			
			// PHA impl
			case 0x48: {
				CPU.last_op = "PHA impl";
				cycles      = 3;
				push( CPU.A );
			} break;
			
			// EOR imm
			case 0x49: {
				CPU.last_op = "EOR imm";
				cycles      = 2;
				EOR( read_imm() );
			} break;
			
			// LSR acc
			case 0x4A: {
				CPU.last_op = "LSR acc";
				cycles      = 2;
				LSR( CPU.A );
			} break;
			
			// JMP abs
			case 0x4C: {
				CPU.last_op = "JMP abs";
				cycles      = 3;
				CPU.PC      = read_addr();
			} break;
			
			// EOR abs
			case 0x4D: {
				CPU.last_op = "EOR abs";
				cycles      = 4;
				EOR( read_abs() );
			} break;
			
			// LSR abs
			case 0x4E: {
				CPU.last_op = "LSR abs";
				cycles      = 6;
				LSR( read_abs() );
			} break;
			
			// BVC rel
			case 0x50: {
				CPU.last_op = "BVC rel";
				cycles      = 2;
				if ( !CPU.read_flag(V) )
					cycles += rel_jmp()? 2 : 1; 
			} break;
			
			// EOR ind,Y
			case 0x51: {
				CPU.last_op = "EOR ind,Y";
				cycles      = 5;
				EOR( read_ind_y(cycles) );
			} break;
			
			// EOR zpg,X
			case 0x55: {
				CPU.last_op = "EOR zpg,X";
				cycles      = 4;
				EOR( read_zpg_x() );
			} break;
			
			// LSR zpg,X
			case 0x56: {
				CPU.last_op = "LSR zpg,X";
				cycles      = 6;
				LSR( read_zpg_x() );
			} break;
			
			// CLI impl
			case 0x58: {
				CPU.last_op = "CLI impl";
				cycles      = 2;
				CPU.clear_flag( I );
			} break;
			
			// EOR abs,Y
			case 0x59: {
				CPU.last_op = "EOR abs,Y";
				cycles      = 4;
				EOR( read_abs_y(cycles) );
			} break;
			
			// EOR abs,X
			case 0x5D: {
				CPU.last_op = "EOR abs,X";
				cycles      = 4;
				EOR( read_abs_x(cycles) );
			} break;
			
			// LSR abs,X
			case 0x5E: {
				CPU.last_op = "LSR abs,X";
				cycles      = 7;
				u8 dummy;
				LSR( read_abs_x(dummy) ); // clean up
			} break;
			
			// RTS impl
			case 0x60: {
				CPU.last_op = "RTS impl";
				cycles      = 6;
				CPU.PC      = pull_addr(); // TODO: verify
			} break;
			
			// ADC ind,X
			case 0x61: {
				CPU.last_op = "ADC ind,X";
				cycles      = 6;
				ADC( read_ind_x() );
			} break;
			
			// ADC zpg
			case 0x65: {
				CPU.last_op = "ADC zpg";
				cycles      = 3;
				ADC( read_zpg() );
			} break;
			
			// ROR zpg
			case 0x66: {
				CPU.last_op = "ROR zpg";
				cycles      = 5;
				ROR( read_zpg() );
			} break;
			
			// PLA impl
			case 0x68: {
				CPU.last_op = "PLA impl";
				cycles      = 4;
				CPU.A       = pull();
				CPU.update_ZN();
			} break;
			
			// ADC imm
			case 0x69: {
				CPU.last_op = "ADC imm";
				cycles      = 2;
				ADC( read_imm() );
			} break;
			
			// ROR acc
			case 0x6A: {
				CPU.last_op = "ROR acc";
				cycles      = 2;
				ROR( CPU.A );
			} break;
			
			// JMP ind
			case 0x6C: { // TODO: verify implementation
				CPU.last_op = "JMP ind";
				cycles      = 5;
				CPU.PC      = read_abs();
				CPU.PC      = read_abs();
				// TODO: reproduce 6502 bug
			} break;
			
			// ADC abs
			case 0x6D: {
				CPU.last_op = "ADC abs";
				cycles      = 4;
				ADC( read_abs() );
			} break;
			
			// ROR abs
			case 0x6E: {
				CPU.last_op = "ROR abs";
				cycles      = 6;
				ROR( read_abs() );
			} break;
			
			// BVS rel
			case 0x70: {
				CPU.last_op = "BVS rel";
				cycles      = 2;
				if ( CPU.read_flag(V) )
					cycles += rel_jmp()? 2 : 1; 
			} break;
			
			// ADC ind,Y
			case 0x71: {
				CPU.last_op = "ADC ind,Y";
				cycles      = 5;
				ADC( read_ind_y(cycles) );
			} break;
			
			// ADC zpg,X
			case 0x75: {
				CPU.last_op = "ADC zpg,X";
				cycles      = 4;
				ADC( read_zpg_x() );
			} break;
			
			// ROR zpg,X
			case 0x76: {
				CPU.last_op = "ROR zpg,X";
				cycles      = 6;
				ROR( read_zpg_x() );
			} break;
			
			// SEI impl
			case 0x78: {
				CPU.last_op = "SEI impl";
				cycles      = 2;
				CPU.set_flag( I );
			} break;
			
			// ADC abs,Y
			case 0x79: {
				CPU.last_op = "ADC abs,Y";
				cycles      = 4;
				ADC( read_abs_y(cycles) );
			} break;
			
			// ADC abs,X
			case 0x7D: {
				CPU.last_op = "ADC abs,X";
				cycles      = 4;
				ADC( read_abs_x(cycles) );
			} break;
			
			// ROR abs,X
			case 0x7E: {
				CPU.last_op = "ROR abs,X";
				cycles      = 7;
				u8 dummy;
				ROR( read_abs_x(dummy) ); // TODO: clean up
			} break;
			
			// STA ind,X
			case 0x81: {
				CPU.last_op  = "STA ind,X";
				cycles       = 6;
				read_ind_x() = CPU.A;
			} break;
			
			// STY zpg
			case 0x84: {
				CPU.last_op = "STY zpg";
				cycles      = 3;
				read_zpg()  = CPU.Y;
			} break;
			
			// STA zpg
			case 0x85: {
				CPU.last_op = "STA zpg";
				cycles      = 3;
				read_zpg()  = CPU.A;
			} break;
			
			// STX zpg
			case 0x86: {
				CPU.last_op = "STX zpg";
				cycles      = 3;
				read_zpg()  = CPU.X;
			} break;
			
			// DEY impl
			case 0x88: {
				CPU.last_op = "DEY impl";
				cycles      = 2;
				DEC( CPU.Y );
			} break;
			
			// TXA impl
			case 0x8A: {
				CPU.last_op = "TXA impl";
				cycles      = 2;
				CPU.A       = CPU.X;
				CPU.update_ZN();
			} break;
			
			// STY abs
			case 0x8C: {
				CPU.last_op = "STY abs";
				cycles      = 4;
				read_abs()  = CPU.Y;
			} break;
			
			// STA abs
			case 0x8D: {
				CPU.last_op = "STA abs";
				cycles      = 4;
				read_abs()  = CPU.A;
			} break;
			
			// STX abs
			case 0x8E: {
				CPU.last_op = "STX abs";
				cycles      = 4;
				read_abs()  = CPU.X;
			} break;
			
			// BCC rel
			case 0x90: {
				CPU.last_op = "BCC rel";
				cycles = 2;
				if ( !CPU.read_flag(C) )
					cycles += rel_jmp()? 2 : 1; 
			} break;
			
			// STA ind,Y
			case 0x91: {
				CPU.last_op = "STA ind,Y";
				cycles      = 6;
				u8 dummy;
				read_ind_y(dummy) = CPU.A; // TODO: clean up
			} break;
			
			// STY zpg,X
			case 0x94: {
				CPU.last_op  = "STY zpg,X";
				cycles       = 4;
				read_zpg_x() = CPU.Y;
			} break;
			
			// STA zpg,X
			case 0x95: {
				CPU.last_op  = "STA zpg,X";
				cycles       = 4;
				read_zpg_x() = CPU.A;
			} break;
			
			// STX zpg,Y
			case 0x96: {
				CPU.last_op  = "STX zpg,Y";
				cycles       = 4;
				read_zpg_y() = CPU.X;
			} break;
			
			// TYA impl
			case 0x98: {
				CPU.last_op = "TYA impl";
				cycles      = 2;
				CPU.A       = CPU.Y;
				CPU.update_ZN();
			} break;
			
			// STA abs,Y
			case 0x99: {
				CPU.last_op = "STA abs,Y";
				u8 dummy;
				read_abs_y(dummy) = CPU.A; // TODO: clean up
				cycles = 5;
			} break;
			
			// TXS impl
			case 0x9A: {
				CPU.last_op = "TXS impl";
				cycles      = 2;
				CPU.SP      = CPU.X;
				CPU.update_ZN();
			} break;
			
			// STA abs,X
			case 0x9D: {
				CPU.last_op = "STA abs,X";
				cycles      = 5;
				u8 dummy;
				read_abs_x(dummy) = CPU.A; // TODO: clean up
			} break;
			
			// LDY imm
			case 0xA0: {
				CPU.last_op = "LDY imm";
				cycles      = 2;
				CPU.Y       = read_imm();
			} break;
			
			// LDA ind,X
			case 0xA1: {
				CPU.last_op = "LDA ind,X";
				cycles      = 6;
				CPU.A       = read_ind_x();
			} break;
			
			// LDX imm
			case 0xA2: {
				CPU.last_op = "LDX imm";
				cycles      = 2;
				CPU.X       = read_imm();
			} break;
			
			// LDY zpg
			case 0xA4: {
				CPU.last_op = "LDY zpg";
				cycles      = 3;
				CPU.Y       = read_zpg();
			} break;
			
			// LDA zpg
			case 0xA5: {
				CPU.last_op = "LDA zpg";
				cycles      = 3;
				CPU.A       = read_zpg();
			} break;
			
			// LDX zpg
			case 0xA6: {
				CPU.last_op = "LDX zpg";
				cycles      = 3;
				CPU.X       = read_zpg();
			} break;
			
			// TAY impl
			case 0xA8: {
				CPU.last_op = "TAY impl";
				cycles      = 2;
				CPU.A       = CPU.Y;
				CPU.update_ZN();
			} break;
			
			// LDA imm
			case 0xA9: {
				CPU.last_op = "LDA imm";
				cycles      = 2;
				CPU.A       = read_imm();
			} break;
			
			// TAX impl
			case 0xAA: {
				CPU.last_op = "TAX impl";
				cycles      = 2;
				CPU.A       = CPU.X;
				CPU.update_ZN();
			} break;
			
			// LDY abs
			case 0xAC: {
				CPU.last_op = "LDY abs";
				cycles      = 4;
				CPU.Y       = read_abs();
			} break;
			
			// LDA abs
			case 0xAD: {
				CPU.last_op = "LDA abs";
				cycles      = 4;
				CPU.A       = read_abs();
			} break;
			
			// LDX abs
			case 0xAE: {
				CPU.last_op = "LDX abs";
				cycles      = 4;
				CPU.X       = read_abs();
			} break;
			
			// BCS rel
			case 0xB0: {
				CPU.last_op = "BCS rel";
				cycles      = 2;
				if ( CPU.read_flag(C) )
					cycles += rel_jmp()? 2 : 1; 
			} break;
			
			// LDA ind,Y
			case 0xB1: {
				CPU.last_op = "LDA ind,Y";
				cycles      = 5;
				CPU.A       = read_ind_y(cycles);
			} break;
			
			// LDY zpg,X
			case 0xB4: {
				CPU.last_op = "LDY zpg,X";
				cycles      = 4;
				CPU.Y       = read_zpg_x();
			} break;
			
			// LDA zpg,X
			case 0xB5: {
				CPU.last_op = "LDA zpg,X";
				cycles      = 4;
				CPU.A       = read_zpg_x();
			} break;
			
			// LDX zpg,Y
			case 0xB6: {
				CPU.last_op = "LDX zpg,Y";
				cycles      = 4;
				CPU.X       = read_zpg_y();
			} break;
			
			// CLV impl
			case 0xB8: {
				CPU.last_op = "CLV impl";
				cycles      = 2;
				CPU.clear_flag( V );
			} break;
			
			// LDA abs,Y
			case 0xB9: {
				CPU.last_op = "LDA abs,Y";
				cycles      = 4;
				CPU.A       = read_abs_y(cycles);
			} break;
			
			// TSX impl
			case 0xBA: {
				CPU.last_op = "TSX impl";
				cycles      = 2;
				CPU.SP      = CPU.X;
				CPU.update_ZN();
			} break;
			
			// LDY abs,X
			case 0xBC: {
				CPU.last_op = "LDY abs,X";
				cycles      = 4;
				CPU.Y       = read_abs_x(cycles);
			} break;
			
			// LDA abs,X
			case 0xBD: {
				CPU.last_op = "LDA abs,X";
				cycles      = 4;
				CPU.A       = read_abs_x(cycles);
			} break;
			
			// LDX abs,Y
			case 0xBE: {
				CPU.last_op = "LDX abs,Y";
				cycles      = 4;
				CPU.X       = read_abs_y(cycles);
			} break;
			
			// CPY imm
			case 0xC0: {
				CPU.last_op = "CPY imm";
				cycles      = 2;
				compare( CPU.Y, read_imm() );
			} break;
			
			// CMP ind,X
			case 0xC1: {
				CPU.last_op = "CMP ind,X";
				cycles      = 6;
				compare( CPU.A, read_ind_x() );
			} break;
			
			// CPY zpg
			case 0xC4: {
				CPU.last_op = "CPY zpg";
				cycles      = 3;
				compare( CPU.Y, read_zpg() );
			} break;
			
			// CMP zpg
			case 0xC5: {
				CPU.last_op = "CMP zpg";
				cycles      = 3;
				compare( CPU.A, read_zpg() );
			} break;
			
			// DEC zpg
			case 0xC6: {
				CPU.last_op = "DEC zpg";
				cycles      = 5;
				DEC( read_zpg() );
			} break;
			
			// INY impl
			case 0xC8: {
				CPU.last_op = "INY impl";
				cycles      = 2;
				INC( CPU.Y );
			} break;
			
			// CMP imm
			case 0xC9: {
				CPU.last_op = "CMP imm";
				cycles      = 2;
				compare( CPU.A, read_imm() );
			} break;
			
			// DEX impl
			case 0xCA: {
				CPU.last_op = "DEX impl";
				cycles      = 2;
				DEC( CPU.X );
			} break;
			
			// CPY abs
			case 0xCC: {
				CPU.last_op = "CPY abs";
				cycles      = 4;
				compare( CPU.Y, read_abs() );
			} break;
			
			// CMP abs
			case 0xCD: {
				CPU.last_op = "CMP abs";
				cycles      = 4;
				compare( CPU.A, read_abs() );
			} break;
			
			// DEC abs
			case 0xCE: {
				CPU.last_op = "DEC abs";
				cycles      = 6;
				DEC( read_abs() );
			} break;
			
			// BNE rel
			case 0xD0: {
				CPU.last_op = "BNE rel";
				cycles      = 2;
				if ( !CPU.read_flag(Z) )
					cycles += rel_jmp()? 2 : 1; 
			} break;
			
			// CMP ind,Y
			case 0xD1: {
				CPU.last_op = "CMP ind,Y";
				cycles      = 5;
				compare( CPU.A, read_ind_y(cycles) );
			} break;
			
			// CMP zpg,X
			case 0xD5: {
				CPU.last_op = "CMP zpg,X";
				cycles      = 4;
				compare( CPU.A, read_zpg_x() );
			} break;
			
			// DEC zpg,X
			case 0xD6: {
				CPU.last_op = "DEC zpg,X";
				cycles      = 6;
				DEC( read_zpg_x() );
			} break;
			
			// CLD impl
			case 0xD8: {
				CPU.last_op = "CLD impl";
				cycles      = 2;
				CPU.clear_flag( D );
			} break;
			
			// CMP abs,Y
			case 0xD9: {
				CPU.last_op = "CMP abs,Y";
				cycles      = 4;
				compare( CPU.A, read_abs_y(cycles) );
			} break;
			
			// CMP abs,X
			case 0xDD: {
				CPU.last_op = "CMP abs,X";
				cycles      = 4;
				compare( CPU.A, read_abs_x(cycles) );
			} break;
			
			// DEC abs,X
			case 0xDE: {
				CPU.last_op = "DEC abs,X";
				cycles      = 7;
				u8 dummy;
				DEC( read_abs_x(dummy) ); // TODO: clean up
			} break;
			
			// CPX imm
			case 0xE0: {
				CPU.last_op = "CPX imm";
				cycles      = 2;
				compare( CPU.X, read_imm() );
			} break;
			
			// SBC ind,X
			case 0xE1: {
				CPU.last_op = "SBC ind,X";
				cycles      = 6;
				SBC( read_ind_x() );
			} break;
			
			// CPX zpg
			case 0xE4: {
				CPU.last_op = "CPX zpg";
				cycles      = 3;
				compare( CPU.X, read_zpg() );
			} break;
			
			// SBC zpg
			case 0xE5: {
				CPU.last_op = "SBC zpg";
				cycles      = 3;
				SBC( read_zpg() );
			} break;
			
			// INC zpg
			case 0xE6: {
				CPU.last_op = "INC zpg";
				cycles      = 5;
				INC( read_zpg() );
			} break;
			
			// INX impl
			case 0xE8: {
				CPU.last_op = "INX impl";
				cycles      = 2;
				INC( CPU.X );
			} break;
			
			// SBC imm
			case 0xE9: {
				CPU.last_op = "SBC imm";
				cycles      = 2;
				SBC( read_imm() );
			} break;
			
			// NOP impl
			case 0xEA: {
				CPU.last_op = "NOP impl";
				cycles      = 2;
			} break;
			
			// CPX abs
			case 0xEC: {
				CPU.last_op = "CPX abs";
				cycles      = 4;
				compare( CPU.X, read_abs() );
			} break;
			
			// SBC abs
			case 0xED: {
				CPU.last_op = "SBC abs";
				cycles      = 4;
				SBC( read_abs() );
			} break;
			
			// INC abs
			case 0xEE: {
				CPU.last_op = "INC abs";
				cycles      = 6;
				INC( read_abs() );
			} break;
			
			//  BEQ rel
			case 0xF0: {
				CPU.last_op = " BEQ rel";
				cycles      = 2;
				if ( CPU.read_flag(Z) )
					cycles += rel_jmp()? 2 : 1; 
			} break;
			
			// SBC ind,Y
			case 0xF1: {
				CPU.last_op = "SBC ind,Y";
				cycles      = 5;
				SBC( read_ind_y(cycles) );
			} break;
			
			// SBC zpg,X
			case 0xF5: {
				CPU.last_op = "SBC zpg,X";
				cycles      = 4;
				SBC( read_zpg_x() );
			} break;
			
			// INC zpg,X
			case 0xF6: {
				CPU.last_op = "INC zpg,X";
				cycles      = 6;
				INC( read_zpg_x() );
			} break;
			
			// SED impl
			case 0xF8: {
				CPU.last_op = "SED impl";
				cycles      = 2;
				CPU.set_flag( D );
			} break;
			
			// SBC abs,Y
			case 0xF9: {
				CPU.last_op = "SBC abs,Y";
				cycles      = 4;
				SBC( read_abs_y(cycles) );
			} break;
			
			// SBC abs,X
			case 0xFD: {
				CPU.last_op = "SBC abs,X";
				cycles      = 4;
				SBC( read_abs_x(cycles) );
			} break;
			
			// INC abs,X
			case 0xFE: {
				CPU.last_op = "INC abs,X";
				cycles      = 6;
				u8 dummy;
				INC( read_abs_x(dummy) ); // TODO: clean up
			} break;
		}
	}
};



struct cpu_debug_15x10_display_t {
	cpu::cpu_t *CPU = nullptr;
	u8 x=0, y=0;
	
	void
	update() noexcept
	{
		using namespace cpu;
		assert( CPU );
		std::printf( "\033[%u;%uH", y, x ); 
		std::printf(
			"╭────────┰─────╮"               "\033[B\033[16D"
			"│PC: %04" PRIX16 "┃ C: %1d│"    "\033[B\033[16D"
			"│SP: %02" PRIX8  "  ┃ Z: %1d│"  "\033[B\033[16D"
			"│ A: %02" PRIX8  "  ┃ I: %1d│"  "\033[B\033[16D"
			"│ X: %02" PRIX8  "  ┃ D: %1d│"  "\033[B\033[16D"
			"│ Y: %02" PRIX8  "  ┃BL: %1d│"  "\033[B\033[16D"
			"┝━━━━━━━━┫BU: %1d│"             "\033[B\033[16D"
			"│%5"   PRIu16 " Hz┃ V: %1d│"    "\033[B\033[16D"
			"│%0.8" PRIu64 "┃ N: %1d│"       "\033[B\033[16D"
			"╰────────┸─────╯",
			CPU->PC, CPU->read_flag( C)? 1:0,
			CPU->SP, CPU->read_flag( Z)? 1:0,
			CPU->A,  CPU->read_flag( I)? 1:0,
			CPU->X,  CPU->read_flag( D)? 1:0,
			CPU->Y,  CPU->read_flag(BL)? 1:0,
			         CPU->read_flag(BU)? 1:0,
			CPU->Hz, CPU->read_flag( V)? 1:0,
			CPU->cc, CPU->read_flag( N)? 1:0
		);
	}
};

struct cpu_debug_82x4_display_t {
	cpu::cpu_t const *CPU = nullptr;
	u8 x=0, y=0;
	
	void
	update() noexcept
	{
		using namespace cpu;
		
		auto constexpr set    = "\e[1;32m1\e[0m";
		auto constexpr unset  = "\e[1;31m0\e[0m";
		
		std::printf( "\033[999;0H" LABEL "Last OP: " BRT_CLR "%-18s\033[0m", CPU->last_op.c_str() ); // TODO: refactor
		
		auto cc_digits = 1;
		auto tmp = CPU->cc;
		while ( tmp /= 10 )
			++cc_digits;
		
		assert( CPU );
		std::printf( "\033[?25l\033[%u;%uH", y, x ); 
		std::printf( 
			"╭────┰────────────────┰──────┰────┰────┰────┰────┰──────────┰──────────────────────╮"  "\033[B\033[84D"
			"│ " LABEL "F:" BORDER " ┃ " LABEL "N V BB D I Z C" BORDER " ┃ " LABEL "PC:" BORDER
			"  ┃ " LABEL "S:" BORDER " ┃ " LABEL "A:" BORDER " ┃ " LABEL "X:" BORDER " ┃ " LABEL "Y:" BORDER
			" ┃ " LABEL "Clk.Spd:" BORDER " ┃ " LABEL "Elapsed Cycle Count:" BORDER " │"  "\033[B\033[84D"
			"│ " BRT_CLR "%02" PRIX8 BORDER " ┃ %s %s %s%s %s %s %s %s ┃ " BRT_CLR "%04" PRIX16 BORDER " ┃ "
			BRT_CLR "%02" PRIX8 BORDER " ┃ " BRT_CLR "%02" PRIX8 BORDER
			" ┃ " BRT_CLR "%02" PRIX8 BORDER " ┃ " BRT_CLR "%02" PRIX8 BORDER " ┃ " BRT_CLR "%5" PRIu16 BORDER " Hz" 
			" ┃ " DIM_CLR "%0*d" BRT_CLR "%" PRIu64 BORDER " │"                   "\033[B\033[84D"
			"╰────┸────────────────┸──────┸────┸────┸────┸────┸──────────┸──────────────────────╯",
			CPU->P,
			CPU->read_flag(N)?set:unset, CPU->read_flag(V)?set:unset, CPU->read_flag(BU)?set:unset, CPU->read_flag(BL)?set:unset,
			CPU->read_flag(D)?set:unset, CPU->read_flag(I)?set:unset, CPU->read_flag( Z)?set:unset, CPU->read_flag( C)?set:unset,
			CPU->PC, CPU->SP, CPU->A, CPU->X, CPU->Y, CPU->Hz, 20-cc_digits, 0, CPU->cc
		);
	}
};


struct ram_page_hex_display_t {
	system_t const *system = nullptr;
	u8  page_no=0;
	u16 x=0, y=0;
	
	void
	update() noexcept
	{
		constexpr u8 width          = 16 * 3 + 6;
		const    u16 page_base_addr = page_no * 0x100;
		assert( system );
		
		std::printf( "\033[?25l\033[%u;%uH"
			"╭─────┰───────────────────────────────────────────────╮"            "\033[B\033[55D"
			"│" LABEL "Pg.%02" PRIX8 BORDER "┃" DIM_CLR "0" BRT_CLR "0 " DIM_CLR "0" BRT_CLR "1 "
			DIM_CLR "0" BRT_CLR "2 " DIM_CLR "0" BRT_CLR "3 " DIM_CLR "0" BRT_CLR "4 " DIM_CLR "0"
			BRT_CLR "5 " DIM_CLR "0" BRT_CLR "6 " DIM_CLR "0" BRT_CLR "7 " DIM_CLR "0" BRT_CLR "8 "
			DIM_CLR "0" BRT_CLR "9 " DIM_CLR "0" BRT_CLR "A " DIM_CLR "0" BRT_CLR "B " DIM_CLR "0"
			BRT_CLR "C " DIM_CLR "0" BRT_CLR "D " DIM_CLR "0" BRT_CLR "E " DIM_CLR "0" BRT_CLR "F"
			BORDER "│"  "\033[B\033[55D"
			"┝━━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┥"            "\033[B\033[55D",
			y, x, page_no
		);
			
		u16 curr_addr = page_base_addr;
		for ( u8 row=0; row<0x10; ++row ) {
			std::printf( "│" DIM_CLR "%02" PRIX8 BRT_CLR "%1" PRIX8 DIM_CLR "0" BORDER " ┃", page_no, row );
			for ( u8 col=0; col<0x10; ++col ) {
				auto const curr_byte = system->RAM[curr_addr++];
				std::printf( "\e[1;%sm%02" PRIX8 "%s", (curr_addr==system->CPU.PC? "40;93" : curr_byte? "32":"31"), curr_byte, col==0xF?"\e[0m":" " );
			}
			std::printf( "│" "\033[B\033[55D" );
		}
		std::printf("╰─────┸───────────────────────────────────────────────╯");
	}
};



int
main( int const argc, char const *const argv[] )
{
	auto system   = system_t {};
	auto terminal = terminal_display_t<80,25> { system.RAM + 0x0200, 2,1 };
	auto dbg_cpu1 = cpu_debug_15x10_display_t { &system.CPU, 124,0 };
	auto dbg_cpu2 = cpu_debug_82x4_display_t  { &system.CPU, 0,27 };
	
	u8 constexpr pg_x=2, pg_y=2;
	ram_page_hex_display_t  hex_page_displays[pg_x * pg_y] {};
	u8 pg_no = 0;
	for ( u8 y=0; y<pg_y; ++y )
		for ( u8 x=0; x<pg_x; ++x )
			hex_page_displays[y*pg_x+x] = ram_page_hex_display_t { &system, pg_no++, u16(x*56U+85U), u16(y*20U+1U) };
	system.CPU.Hz = argc == 2? std::atoi(argv[1]) : 500;
	auto txt      = " xx Hello world!";
	std::memcpy( (void*)terminal.block_start, txt, 16 );
	
	// Test program; kinda broken (TODO: fix)
	u8 prg[] = {
/* 0300 MAIN   LDX #$00    */  0xA2, 0x00,
/* 0302 LOOP1  LDY #$00    */  0xA0, 0x00,
/* 0304 LOOP2  TXA         */  0x8A,
/* 0305        PHA         */  0x48,
/* 0306        TYA         */  0x98,
/* 0307        PHA         */  0x48,
/* 0308        JSR PRINT   */  0x20, 0x1F, 0x03,
/* 030B        PLA         */  0x68,
/* 030C        TAY         */  0xA8,
/* 030D        PLA         */  0x68,
/* 030E        TAX         */  0xAA,
/* 030F        INY         */  0xC8,
/* 0310        TYA         */  0x98,
/* 0311        SBC #$0A    */  0xE9, 0x0A,
/* 0313        BNE LOOP2   */  0xD0, 0xEF,
/* 0315        INX         */  0xE8,
/* 0316        TXA         */  0x8A,
/* 0317        SBC #$0A    */  0xE9, 0x0A,
/* 0319        BNE LOOP1   */  0xD0, 0xE7,
/* 031B END    NOP         */  0xEA,
/* 031C        JMP END     */  0x4C, 0x1B, 0x03,
/*-------------------------*/  
/* 031F PRINT  STY *$00    */  0x84, 0x00,
/* 0321        TXA         */  0x8A,
/* 0322        SED         */  0xF8,
/* 0323        ADC *$00    */  0x65, 0x00,
/* 0325        CLD         */  0xD8,
/* 0326        STA *$00    */  0x85, 0x00,
/* 0328        LSR A       */  0x4A,
/* 0329        LSR A       */  0x4A,
/* 032A        LSR A       */  0x4A,
/* 032B        LSR A       */  0x4A,
/* 032C        ADC #$30    */  0x69, 0x30,
/* 032E        STA $0201   */  0x8D, 0x01, 0x02,
/* 0331        LDA *$00    */  0xA5, 0x00,
/* 0333        AND #$0F    */  0x29, 0x0F,
/* 0335        ADC #$30    */  0x69, 0x30,
/* 0337        STA $0202   */  0x8D, 0x02, 0x02,
/* 033A        RTS         */  0x60
	};
	
	std::memcpy( (void*)(system.RAM+0x0300), prg, sizeof(prg) );
	system.CPU.PC = 0x0300;
	
	f32 constexpr draw_frame_window_ms = 1000.0f / 30;
	
	f32 time_since_draw_ms = draw_frame_window_ms,
	    frame_time_elapsed = .0f;
	while (true) {
		frame_time_elapsed = timer();
			
		system.update();
		
		if ( time_since_draw_ms >= draw_frame_window_ms ) {
			time_since_draw_ms -= draw_frame_window_ms;
			terminal.update();
			dbg_cpu2.update();
			for ( auto &display: hex_page_displays )
				display.update();
			//while ( system.CPU.last_op == "RTS impl" ); // TEMP breaker
		}
		
		frame_time_elapsed  = timer() - frame_time_elapsed;
		time_since_draw_ms += frame_time_elapsed;
	}
}

