#include <cstdint>
#include <cinttypes>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <string>
#include <chrono>
#include <thread>
#include <unordered_map>

using namespace::std::literals;

#include <unistd.h> // for usleep
//#include <ctime>    // for dt

// TODO: suffix with CLR?
#define LABEL   "\e[0;1;33m"
#define BORDER  "\e[0;1m"
#define DIM_CLR "\e[0;90m"
#define BRT_CLR "\e[0;97m"

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
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
// TODO: keep track of last written + clear later

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
		while (1) {
			system("stty raw");
			*port = getchar(); 
			// terminate when "." is pressed
			system("stty cooked");
			system("clear");
			std::printf( "\033[99;0H" LABEL "Last keypress: " BRT_CLR "'%c'\033[0m", *port ); // TODO: refactor
			if ( *port == '.' ) {
				system("stty cooked");
				exit(0);
			}  
		}
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
		u64 cc              =  0; // cycle count
		u32 last_write_addr = -1;
		u16 last_push_addr  = -1;
		
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

struct system_t
{
	
	cpu::cpu_t CPU; // MOS Technology 6502
	
	//u8 RAM[ram_kb];
	u8 RAM[0xFFFF+1];
	
	struct history_data_t {
		u16 addr=0x0000;
		u8  op=0xEA, cycles=0, arg1=0, arg2=0;
	};
	
	static constexpr u64 history_capacity = 8192;
	struct history_stack_t {
	private:
		history_data_t  _data[history_capacity];
		u64             _size = 0;
		u16             _head = 0;
	public:
		inline void
		push( u16 const addr, u8 const op, u8 const cycles, u8 const arg1=0, u8 const arg2=0 ) noexcept
		{
			_data[_head] = { addr, op, cycles, arg1, arg2 };
			_head        = (_head+1) % history_capacity;
			if ( _size < history_capacity )
				++_size;
		}
		
		[[nodiscard]] inline auto
		pop() noexcept -> history_data_t
		{
			assert( _size and "Can't pop() empty stack!" );
			_head = (history_capacity+_head-1) % history_capacity;
			--_size;
			return _data[_head];
		}
		
		[[nodiscard]] inline auto
		peek( u64 const offset=0 ) const noexcept -> history_data_t
		{
			assert( offset < _size and "Trying to peek() out of bounds!" );
			return _data[(history_capacity+_head-offset-1) % history_capacity];
		}
		
		[[nodiscard]] inline auto
		size() const noexcept -> u64
		{
			return _size;
		}
	} history;
	
	struct op_info_t {
		char const format[64];
		u8   const bytes;
	};
	
	// TODO: replace with a ADS that isn't absolute shit
	static const inline auto op_meta_tbl = std::unordered_map<u8,op_info_t> {
	//   __key__  ____________value___
	//   OP code { ASM format,  bytes }
		{ 0x00,   { "BRK         ", 1     } },
		{ 0x01,   { "ORA (bb,X)  ", 2     } },
		{ 0x05,   { "ORA *ll     ", 2     } },
		{ 0x06,   { "ASL *ll     ", 2     } },
		{ 0x08,   { "PHP         ", 1     } },
		{ 0x09,   { "ORA #bb     ", 2     } },
		{ 0x0A,   { "ASL A       ", 1     } },
		{ 0x0D,   { "ORA hhll    ", 3     } },
		{ 0x0E,   { "ASL hhll    ", 3     } },
		{ 0x10,   { "BPL bb      ", 2     } },
		{ 0x11,   { "ORA (ll),Y  ", 2     } },
		{ 0x15,   { "ORA *ll,X   ", 2     } },
		{ 0x16,   { "ASL *ll,X   ", 2     } },
		{ 0x18,   { "CLC         ", 1     } },
		{ 0x19,   { "ORA hhll,Y  ", 3     } },
		{ 0x1D,   { "ORA hhll,X  ", 3     } },
		{ 0x1E,   { "ASL hhll,X  ", 3     } },
		{ 0x20,   { "JSR hhll    ", 3     } },
		{ 0x21,   { "AND (bb,X)  ", 2     } },
		{ 0x24,   { "BIT *ll     ", 2     } },
		{ 0x25,   { "AND *ll     ", 2     } },
		{ 0x26,   { "ROL *ll     ", 2     } },
		{ 0x28,   { "PLP         ", 1     } },
		{ 0x29,   { "AND #bb     ", 2     } },
		{ 0x2A,   { "ROL A       ", 1     } },
		{ 0x2C,   { "BIT hhll    ", 3     } },
		{ 0x2D,   { "AND hhll    ", 3     } },
		{ 0x2E,   { "ROL hhll    ", 3     } },
		{ 0x30,   { "BMI bb      ", 2     } },
		{ 0x31,   { "AND (ll),Y  ", 2     } },
		{ 0x35,   { "AND *ll,X   ", 2     } },
		{ 0x36,   { "ROL *ll,X   ", 2     } },
		{ 0x38,   { "SEC         ", 1     } },
		{ 0x39,   { "AND hhll,Y  ", 3     } },
		{ 0x3D,   { "AND hhll,X  ", 3     } },
		{ 0x3E,   { "ROL hhll,X  ", 3     } },
		{ 0x40,   { "RTI         ", 1     } },
		{ 0x41,   { "EOR (bb,X)  ", 2     } },
		{ 0x45,   { "EOR *ll     ", 2     } },
		{ 0x46,   { "LSR *ll     ", 2     } },
		{ 0x48,   { "PHA         ", 1     } },
		{ 0x49,   { "EOR #bb     ", 2     } },
		{ 0x4A,   { "LSR A       ", 1     } },
		{ 0x4C,   { "JMP hhll    ", 3     } },
		{ 0x4D,   { "EOR hhll    ", 3     } },
		{ 0x4E,   { "LSR hhll    ", 3     } },
		{ 0x50,   { "BVC bb      ", 2     } },
		{ 0x51,   { "EOR (ll),Y  ", 2     } },
		{ 0x55,   { "EOR *ll,X   ", 2     } },
		{ 0x56,   { "LSR *ll,X   ", 2     } },
		{ 0x58,   { "CLI         ", 1     } },
		{ 0x59,   { "EOR hhll,Y  ", 3     } },
		{ 0x5D,   { "EOR hhll,X  ", 3     } },
		{ 0x5E,   { "LSR hhll,X  ", 3     } },
		{ 0x60,   { "RTS         ", 1     } },
		{ 0x61,   { "ADC (bb,X)  ", 2     } },
		{ 0x65,   { "ADC *ll     ", 2     } },
		{ 0x66,   { "ROR *ll     ", 2     } },
		{ 0x68,   { "PLA         ", 1     } },
		{ 0x69,   { "ADC #bb     ", 2     } },
		{ 0x6A,   { "ROR A       ", 1     } },
		{ 0x6C,   { "JMP ind     ", 2     } },
		{ 0x6D,   { "ADC hhll    ", 3     } },
		{ 0x6E,   { "ROR hhll    ", 3     } },
		{ 0x70,   { "BVS bb      ", 2     } },
		{ 0x71,   { "ADC (ll),Y  ", 2     } },
		{ 0x75,   { "ADC *ll,X   ", 2     } },
		{ 0x76,   { "ROR *ll,X   ", 2     } },
		{ 0x78,   { "SEI         ", 1     } },
		{ 0x79,   { "ADC hhll,Y  ", 3     } },
		{ 0x7D,   { "ADC hhll,X  ", 3     } },
		{ 0x7E,   { "ROR hhll,X  ", 3     } },
		{ 0x81,   { "STA (bb,X)  ", 2     } },
		{ 0x84,   { "STY *ll     ", 2     } },
		{ 0x85,   { "STA *ll     ", 2     } },
		{ 0x86,   { "STX *ll     ", 2     } },
		{ 0x88,   { "DEY         ", 1     } },
		{ 0x8A,   { "TXA         ", 1     } },
		{ 0x8C,   { "STY hhll    ", 3     } },
		{ 0x8D,   { "STA hhll    ", 3     } },
		{ 0x8E,   { "STX hhll    ", 3     } },
		{ 0x90,   { "BCC bb      ", 2     } },
		{ 0x91,   { "STA (ll),Y  ", 2     } },
		{ 0x94,   { "STY *ll,X   ", 2     } },
		{ 0x95,   { "STA *ll,X   ", 2     } },
		{ 0x96,   { "STX *ll,Y   ", 2     } },
		{ 0x98,   { "TYA         ", 1     } },
		{ 0x99,   { "STA hhll,Y  ", 3     } },
		{ 0x9A,   { "TXS         ", 1     } },
		{ 0x9D,   { "STA hhll,X  ", 3     } },
		{ 0xA0,   { "LDY #bb     ", 2     } },
		{ 0xA1,   { "LDA (bb,X)  ", 2     } },
		{ 0xA2,   { "LDX #bb     ", 2     } },
		{ 0xA4,   { "LDY *ll     ", 2     } },
		{ 0xA5,   { "LDA *ll     ", 2     } },
		{ 0xA6,   { "LDX *ll     ", 2     } },
		{ 0xA8,   { "TAY         ", 1     } },
		{ 0xA9,   { "LDA #bb     ", 2     } },
		{ 0xAA,   { "TAX         ", 1     } },
		{ 0xAC,   { "LDY hhll    ", 3     } },
		{ 0xAD,   { "LDA hhll    ", 3     } },
		{ 0xAE,   { "LDX hhll    ", 3     } },
		{ 0xB0,   { "BCS bb      ", 2     } },
		{ 0xB1,   { "LDA (ll),Y  ", 2     } },
		{ 0xB4,   { "LDY *ll,X   ", 2     } },
		{ 0xB5,   { "LDA *ll,X   ", 2     } },
		{ 0xB6,   { "LDX *ll,Y   ", 2     } },
		{ 0xB8,   { "CLV         ", 1     } },
		{ 0xB9,   { "LDA hhll,Y  ", 2     } },
		{ 0xBA,   { "TSX         ", 1     } },
		{ 0xBC,   { "LDY hhll,X  ", 3     } },
		{ 0xBD,   { "LDA hhll,X  ", 3     } },
		{ 0xBE,   { "LDX hhll,Y  ", 3     } },
		{ 0xC0,   { "CPY #bb     ", 2     } },
		{ 0xC1,   { "CMP (bb,X)  ", 2     } },
		{ 0xC4,   { "CPY *ll     ", 2     } },
		{ 0xC5,   { "CMP *ll     ", 2     } },
		{ 0xC6,   { "DEC *ll     ", 2     } },
		{ 0xC8,   { "INY         ", 1     } },
		{ 0xC9,   { "CMP #bb     ", 2     } },
		{ 0xCA,   { "DEX         ", 1     } },
		{ 0xCC,   { "CPY hhll    ", 3     } },
		{ 0xCD,   { "CMP hhll    ", 3     } },
		{ 0xCE,   { "DEC hhll    ", 3     } },
		{ 0xD0,   { "BNE bb      ", 2     } },
		{ 0xD1,   { "CMP (ll),Y  ", 2     } },
		{ 0xD5,   { "CMP *ll,X   ", 2     } },
		{ 0xD6,   { "DEC *ll,X   ", 2     } },
		{ 0xD8,   { "CLD         ", 1     } },
		{ 0xD9,   { "CMP hhll,Y  ", 3     } },
		{ 0xDD,   { "CMP hhll,X  ", 3     } },
		{ 0xDE,   { "DEC hhll,X  ", 3     } },
		{ 0xE0,   { "CPX #bb     ", 2     } },
		{ 0xE1,   { "SBC (bb,X)  ", 2     } },
		{ 0xE4,   { "CPX *ll     ", 2     } },
		{ 0xE5,   { "SBC *ll     ", 2     } },
		{ 0xE6,   { "INC *ll     ", 2     } },
		{ 0xE8,   { "INX         ", 1     } },
		{ 0xE9,   { "SBC #bb     ", 2     } },
		{ 0xEA,   { "NOP         ", 1     } },
		{ 0xEC,   { "CPX hhll    ", 3     } },
		{ 0xED,   { "SBC hhll    ", 3     } },
		{ 0xEE,   { "INC hhll    ", 3     } },
		{ 0xF0,   { "BEQ bb      ", 2     } },
		{ 0xF1,   { "SBC (ll),Y  ", 2     } },
		{ 0xF5,   { "SBC *ll,X   ", 2     } },
		{ 0xF6,   { "INC *ll,X   ", 2     } },
		{ 0xF8,   { "SED         ", 1     } },
		{ 0xF9,   { "SBC hhll,Y  ", 3     } },
		{ 0xFD,   { "SBC hhll,X  ", 3     } },
		{ 0xFE,   { "INC hhll,X  ", 3     } }
	};
	
	inline void
	push( u8 const byte ) noexcept
	{
		CPU.last_push_addr = CPU.SP;
		// TODO: optional overflow guard? wrap?
		RAM[0x0100 + CPU.SP++] = byte; // TODO: verify
	}
	
	inline void
	push_addr( u16 const addr ) noexcept
	{
		CPU.last_push_addr = CPU.SP;
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
			u8  tmp_l = (CPU.A&0x0F) + (byte&0x0F);
			if ( tmp_l > 0x09 )
				tmp_l += 0x06;
			u16 tmp = (CPU.A&0xF0) + (byte&0xF0) + tmp_l;
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
		
		while (1) {
			// TODO: fix cycle elapse pre OP side-effects instead of post
			
			u8 const instruction      = RAM[CPU.PC++];
			u8 const instruction_size = op_meta_tbl.at(instruction).bytes;
			u8 const arg1             = instruction_size < 2? 0x00 : RAM[CPU.PC  ];
			u8 const arg2             = instruction_size < 3? 0x00 : RAM[CPU.PC+1];
			
			switch (instruction)
			{
				// BRK impl
				case 0x00: {
					cycles = 7;
					CPU.PC = (RAM[0xFFFF] << 8) + RAM[0xFFFE]; // TODO: verify
					CPU.set_flag( I, 1 );
					push( CPU.PC );
					push( CPU.P  );
					while(1); // TODO: remove
				} break; 
				
				// ORA ind,X
				case 0x01: {
					cycles = 6;
					ORA( read_ind_x() );
				} break;
				
				// ORA zpg
				case 0x05: {
					cycles = 3;
					ORA( read_zpg() );
				} break;
				
				// ASL zpg
				case 0x06: {
					cycles = 5;
					ASL( read_zpg() );
				} break;
				
				// PHP impl
				case 0x08: {
					cycles = 3;
					push( CPU.P );
				} break;
				
				// ORA imm
				case 0x09: {
					cycles = 2;
					ORA( read_imm() );
				} break;
				
				// ASL acc
				case 0x0A: {
					cycles = 2;
					ASL( CPU.A );
				} break;
				
				// ORA abs
				case 0x0D: {
					cycles = 4;
					ORA( read_abs() );
				} break;
				
				// ASL abs
				case 0x0E: {
					cycles = 6;
					ASL( read_abs() );
				} break;
				
				// BPL rel
				case 0x10: {
					cycles = 2;
					if ( !CPU.read_flag(N) )
						cycles += rel_jmp()? 2 : 1; 
				} break;
				
				// ORA ind,Y
				case 0x11: {
					cycles = 5;
					ORA( read_ind_y(cycles) );
				} break;
				
				// ORA zpg,X
				case 0x15: {
					cycles = 4;
					ORA( read_zpg_x() );
				} break;
				
				// ASL zpg,X
				case 0x16: {
					cycles = 6;
					ASL( read_zpg_x() );
				} break;
				
				// CLC impl
				case 0x18: {
					cycles = 2;
					CPU.clear_flag( C );
				} break;
				
				// ORA abs,Y
				case 0x19: {
					cycles = 4;
					ORA( read_abs_y(cycles) );
				} break;
				
				// ORA abs,X
				case 0x1D: {
					cycles = 4;
					ORA( read_abs_x(cycles) );
				} break;
				
				// ASL abs,X
				case 0x1E: {
					cycles = 7;
					u8 dummy;
					ASL( read_abs_x(dummy) ); // TODO: clean up
				} break;
				
				// JSR abs
				case 0x20: {
					cycles = 6;
					push_addr( CPU.PC+2 );
					CPU.PC      = read_addr();
				} break;
				
				// AND ind,X
				case 0x21: {
					cycles = 6;
					AND( read_ind_x() );
				} break;
				
				// BIT zpg
				case 0x24: {
					cycles   = 3;
					auto tmp = CPU.A & read_zpg();
					CPU.set_flag( N, tmp>>7 );
					CPU.set_flag( V, (tmp>>6)&1 );
				} break;
				
				// AND zpg
				case 0x25: {
					cycles = 3;
					AND( read_zpg() );
				} break;
				
				// ROL zpg
				case 0x26: {
					cycles = 5;
					ROL( read_zpg() );
				} break;
				
				// PLP impl
				case 0x28: {
					// TODO: verify break restoration veracity
					cycles = 4;
					CPU.P  = pull();
				} break;
				
				// AND imm
				case 0x29: {
					cycles = 2;
					AND( read_imm() );
				} break;
				
				// ROL acc
				case 0x2A: {
					cycles = 2;
					ROL( CPU.A );
				} break;
				
				// BIT abs
				case 0x2C: {
					
				} break;
				
				// AND abs
				case 0x2D: {
					cycles = 4;
					AND( read_abs() );
				} break;
				
				// ROL abs
				case 0x2E: {
					cycles = 6;
					ROL( read_abs() );
				} break;
				
				// BMI rel
				case 0x30: {
					cycles = 2;
					if ( CPU.read_flag(N) )
						cycles += rel_jmp()? 2 : 1; 
				} break;
				
				// AND ind,Y
				case 0x31: {
					cycles = 5;
					AND( read_ind_y(cycles) );
				} break;
				
				// AND zpg,X
				case 0x35: {
					cycles = 4;
					AND( read_zpg_x() );
				} break;
				
				// ROL zpg,X
				case 0x36: {
					cycles = 6;
					ROL( read_zpg_x() );
				} break;
				
				// SEC impl
				case 0x38: {
					cycles = 2;
					CPU.set_flag( C );
				} break;
				
				// AND abs,Y
				case 0x39: {
					cycles = 4;
					AND( read_abs_y(cycles) );
				} break;
				
				// AND abs,X
				case 0x3D: {
					cycles = 4;
					AND( read_abs_x(cycles) );
				} break;
				
				// ROL abs,X
				case 0x3E: {
					cycles = 7;
					u8 dummy;
					ROL( read_abs_x(dummy) ); // TODO: clean up
				} break;
				
				// RTI impl
				case 0x40: { // TODO: verify endianness
					cycles   = 6;
					CPU.P    = pull();
					CPU.PC   = pull(); // MSB
					CPU.PC <<= 8;
					CPU.PC  += pull(); // LSB
				} break;
				
				// EOR ind,X
				case 0x41: {
					cycles = 6;
					EOR( read_ind_x() );
				} break;
				
				// EOR zpg
				case 0x45: {
					cycles = 3;
					EOR( read_zpg() );
				} break;
				
				// LSR zpg
				case 0x46: {
					cycles = 5;
					LSR( read_zpg() );
				} break;
				
				// PHA impl
				case 0x48: {
					cycles = 3;
					push( CPU.A );
				} break;
				
				// EOR imm
				case 0x49: {
					cycles = 2;
					EOR( read_imm() );
				} break;
				
				// LSR acc
				case 0x4A: {
					cycles = 2;
					LSR( CPU.A );
				} break;
				
				// JMP abs
				case 0x4C: {
					cycles = 3;
					CPU.PC = read_addr();
				} break;
				
				// EOR abs
				case 0x4D: {
					cycles = 4;
					EOR( read_abs() );
				} break;
				
				// LSR abs
				case 0x4E: {
					cycles = 6;
					LSR( read_abs() );
				} break;
				
				// BVC rel
				case 0x50: {
					cycles = 2;
					if ( !CPU.read_flag(V) )
						cycles += rel_jmp()? 2 : 1; 
				} break;
				
				// EOR ind,Y
				case 0x51: {
					cycles = 5;
					EOR( read_ind_y(cycles) );
				} break;
				
				// EOR zpg,X
				case 0x55: {
					cycles = 4;
					EOR( read_zpg_x() );
				} break;
				
				// LSR zpg,X
				case 0x56: {
					cycles = 6;
					LSR( read_zpg_x() );
				} break;
				
				// CLI impl
				case 0x58: {
					cycles = 2;
					CPU.clear_flag( I );
				} break;
				
				// EOR abs,Y
				case 0x59: {
					cycles = 4;
					EOR( read_abs_y(cycles) );
				} break;
				
				// EOR abs,X
				case 0x5D: {
					cycles = 4;
					EOR( read_abs_x(cycles) );
				} break;
				
				// LSR abs,X
				case 0x5E: {
					cycles = 7;
					u8 dummy;
					LSR( read_abs_x(dummy) ); // clean up
				} break;
				
				// RTS impl
				case 0x60: {
					cycles = 6;
					CPU.PC = pull_addr(); // TODO: verify
				} break;
				
				// ADC ind,X
				case 0x61: {
					cycles = 6;
					ADC( read_ind_x() );
				} break;
				
				// ADC zpg
				case 0x65: {
					cycles = 3;
					ADC( read_zpg() );
				} break;
				
				// ROR zpg
				case 0x66: {
					cycles = 5;
					ROR( read_zpg() );
				} break;
				
				// PLA impl
				case 0x68: {
					cycles = 4;
					CPU.A  = pull();
					CPU.update_ZN();
				} break;
				
				// ADC imm
				case 0x69: {
					cycles = 2;
					ADC( read_imm() );
				} break;
				
				// ROR acc
				case 0x6A: {
					cycles = 2;
					ROR( CPU.A );
				} break;
				
				// JMP ind
				case 0x6C: { // TODO: verify implementation
					cycles = 5;
					CPU.PC = read_abs();
					CPU.PC = read_abs();
					// TODO: reproduce 6502 bug
				} break;
				
				// ADC abs
				case 0x6D: {
					cycles = 4;
					ADC( read_abs() );
				} break;
				
				// ROR abs
				case 0x6E: {
					cycles = 6;
					ROR( read_abs() );
				} break;
				
				// BVS rel
				case 0x70: {
					cycles = 2;
					if ( CPU.read_flag(V) )
						cycles += rel_jmp()? 2 : 1; 
				} break;
				
				// ADC ind,Y
				case 0x71: {
					cycles = 5;
					ADC( read_ind_y(cycles) );
				} break;
				
				// ADC zpg,X
				case 0x75: {
					cycles = 4;
					ADC( read_zpg_x() );
				} break;
				
				// ROR zpg,X
				case 0x76: {
					cycles = 6;
					ROR( read_zpg_x() );
				} break;
				
				// SEI impl
				case 0x78: {
					cycles = 2;
					CPU.set_flag( I );
				} break;
				
				// ADC abs,Y
				case 0x79: {
					cycles = 4;
					ADC( read_abs_y(cycles) );
				} break;
				
				// ADC abs,X
				case 0x7D: {
					cycles = 4;
					ADC( read_abs_x(cycles) );
				} break;
				
				// ROR abs,X
				case 0x7E: {
					cycles = 7;
					u8 dummy;
					ROR( read_abs_x(dummy) ); // TODO: clean up
				} break;
				
				// STA ind,X
				case 0x81: {
					cycles       = 6;
					read_ind_x() = CPU.A;
				} break;
				
				// STY zpg
				case 0x84: {
					cycles     = 3;
					read_zpg() = CPU.Y;
				} break;
				
				// STA zpg
				case 0x85: {
					cycles     = 3;
					read_zpg() = CPU.A;
				} break;
				
				// STX zpg
				case 0x86: {
					cycles     = 3;
					read_zpg() = CPU.X;
				} break;
				
				// DEY impl
				case 0x88: {
					cycles = 2;
					DEC( CPU.Y );
				} break;
				
				// TXA impl
				case 0x8A: {
					cycles = 2;
					CPU.A  = CPU.X;
					CPU.update_ZN();
				} break;
				
				// STY abs
				case 0x8C: {
					cycles     = 4;
					read_abs() = CPU.Y;
				} break;
				
				// STA abs
				case 0x8D: {
					cycles     = 4;
					read_abs() = CPU.A;
				} break;
				
				// STX abs
				case 0x8E: {
					cycles     = 4;
					read_abs() = CPU.X;
				} break;
				
				// BCC rel
				case 0x90: {
					cycles = 2;
					if ( !CPU.read_flag(C) )
						cycles += rel_jmp()? 2 : 1; 
				} break;
				
				// STA ind,Y
				case 0x91: {
					cycles = 6;
					u8 dummy;
					read_ind_y(dummy) = CPU.A; // TODO: clean up
				} break;
				
				// STY zpg,X
				case 0x94: {
					cycles       = 4;
					read_zpg_x() = CPU.Y;
				} break;
				
				// STA zpg,X
				case 0x95: {
					cycles       = 4;
					read_zpg_x() = CPU.A;
				} break;
				
				// STX zpg,Y
				case 0x96: {
					cycles       = 4;
					read_zpg_y() = CPU.X;
				} break;
				
				// TYA impl
				case 0x98: {
					cycles = 2;
					CPU.A  = CPU.Y;
					CPU.update_ZN();
				} break;
				
				// STA abs,Y
				case 0x99: {
					u8 dummy;
					read_abs_y(dummy) = CPU.A; // TODO: clean up
					cycles = 5;
				} break;
				
				// TXS impl
				case 0x9A: {
					cycles = 2;
					CPU.SP = CPU.X;
					CPU.update_ZN();
				} break;
				
				// STA abs,X
				case 0x9D: {
					cycles = 5;
					u8 dummy;
					read_abs_x(dummy) = CPU.A; // TODO: clean up
				} break;
				
				// LDY imm
				case 0xA0: {
					cycles = 2;
					CPU.Y  = read_imm();
				} break;
				
				// LDA ind,X
				case 0xA1: {
					cycles = 6;
					CPU.A  = read_ind_x();
				} break;
				
				// LDX imm
				case 0xA2: {
					cycles = 2;
					CPU.X  = read_imm();
				} break;
				
				// LDY zpg
				case 0xA4: {
					cycles = 3;
					CPU.Y  = read_zpg();
				} break;
				
				// LDA zpg
				case 0xA5: {
					cycles = 3;
					CPU.A  = read_zpg();
				} break;
				
				// LDX zpg
				case 0xA6: {
					cycles = 3;
					CPU.X  = read_zpg();
				} break;
				
				// TAY impl
				case 0xA8: {
					cycles = 2;
					CPU.A  = CPU.Y;
					CPU.update_ZN();
				} break;
				
				// LDA imm
				case 0xA9: {
					cycles = 2;
					CPU.A  = read_imm();
				} break;
				
				// TAX impl
				case 0xAA: {
					cycles = 2;
					CPU.A  = CPU.X;
					CPU.update_ZN();
				} break;
				
				// LDY abs
				case 0xAC: {
					cycles = 4;
					CPU.Y  = read_abs();
				} break;
				
				// LDA abs
				case 0xAD: {
					cycles = 4;
					CPU.A  = read_abs();
				} break;
				
				// LDX abs
				case 0xAE: {
					cycles = 4;
					CPU.X  = read_abs();
				} break;
				
				// BCS rel
				case 0xB0: {
					cycles = 2;
					if ( CPU.read_flag(C) )
						cycles += rel_jmp()? 2 : 1; 
				} break;
				
				// LDA ind,Y
				case 0xB1: {
					cycles = 5;
					CPU.A  = read_ind_y(cycles);
				} break;
				
				// LDY zpg,X
				case 0xB4: {
					cycles = 4;
					CPU.Y  = read_zpg_x();
				} break;
				
				// LDA zpg,X
				case 0xB5: {
					cycles = 4;
					CPU.A  = read_zpg_x();
				} break;
				
				// LDX zpg,Y
				case 0xB6: {
					cycles = 4;
					CPU.X  = read_zpg_y();
				} break;
				
				// CLV impl
				case 0xB8: {
					cycles = 2;
					CPU.clear_flag( V );
				} break;
				
				// LDA abs,Y
				case 0xB9: {
					cycles = 4;
					CPU.A  = read_abs_y(cycles);
				} break;
				
				// TSX impl
				case 0xBA: {
					cycles = 2;
					CPU.SP = CPU.X;
					CPU.update_ZN();
				} break;
				
				// LDY abs,X
				case 0xBC: {
					cycles = 4;
					CPU.Y  = read_abs_x(cycles);
				} break;
				
				// LDA abs,X
				case 0xBD: {
					cycles = 4;
					CPU.A  = read_abs_x(cycles);
				} break;
				
				// LDX abs,Y
				case 0xBE: {
					cycles = 4;
					CPU.X  = read_abs_y(cycles);
				} break;
				
				// CPY imm
				case 0xC0: {
					cycles = 2;
					compare( CPU.Y, read_imm() );
				} break;
				
				// CMP ind,X
				case 0xC1: {
					cycles = 6;
					compare( CPU.A, read_ind_x() );
				} break;
				
				// CPY zpg
				case 0xC4: {
					cycles = 3;
					compare( CPU.Y, read_zpg() );
				} break;
				
				// CMP zpg
				case 0xC5: {
					cycles = 3;
					compare( CPU.A, read_zpg() );
				} break;
				
				// DEC zpg
				case 0xC6: {
					cycles = 5;
					DEC( read_zpg() );
				} break;
				
				// INY impl
				case 0xC8: {
					cycles = 2;
					INC( CPU.Y );
				} break;
				
				// CMP imm
				case 0xC9: {
					cycles = 2;
					compare( CPU.A, read_imm() );
				} break;
				
				// DEX impl
				case 0xCA: {
					cycles = 2;
					DEC( CPU.X );
				} break;
				
				// CPY abs
				case 0xCC: {
					cycles = 4;
					compare( CPU.Y, read_abs() );
				} break;
				
				// CMP abs
				case 0xCD: {
					cycles = 4;
					compare( CPU.A, read_abs() );
				} break;
				
				// DEC abs
				case 0xCE: {
					cycles = 6;
					DEC( read_abs() );
				} break;
				
				// BNE rel
				case 0xD0: {
					cycles = 2;
					if ( !CPU.read_flag(Z) )
						cycles += rel_jmp()? 2 : 1; 
				} break;
				
				// CMP ind,Y
				case 0xD1: {
					cycles = 5;
					compare( CPU.A, read_ind_y(cycles) );
				} break;
				
				// CMP zpg,X
				case 0xD5: {
					cycles = 4;
					compare( CPU.A, read_zpg_x() );
				} break;
				
				// DEC zpg,X
				case 0xD6: {
					cycles = 6;
					DEC( read_zpg_x() );
				} break;
				
				// CLD impl
				case 0xD8: {
					cycles = 2;
					CPU.clear_flag( D );
				} break;
				
				// CMP abs,Y
				case 0xD9: {
					cycles = 4;
					compare( CPU.A, read_abs_y(cycles) );
				} break;
				
				// CMP abs,X
				case 0xDD: {
					cycles = 4;
					compare( CPU.A, read_abs_x(cycles) );
				} break;
				
				// DEC abs,X
				case 0xDE: {
					cycles = 7;
					u8 dummy;
					DEC( read_abs_x(dummy) ); // TODO: clean up
				} break;
				
				// CPX imm
				case 0xE0: {
					cycles = 2;
					compare( CPU.X, read_imm() );
				} break;
				
				// SBC ind,X
				case 0xE1: {
					cycles = 6;
					SBC( read_ind_x() );
				} break;
				
				// CPX zpg
				case 0xE4: {
					cycles = 3;
					compare( CPU.X, read_zpg() );
				} break;
				
				// SBC zpg
				case 0xE5: {
					cycles = 3;
					SBC( read_zpg() );
				} break;
				
				// INC zpg
				case 0xE6: {
					cycles = 5;
					INC( read_zpg() );
				} break;
				
				// INX impl
				case 0xE8: {
					cycles = 2;
					INC( CPU.X );
				} break;
				
				// SBC imm
				case 0xE9: {
					cycles = 2;
					SBC( read_imm() );
				} break;
				
				// NOP impl
				case 0xEA: {
					cycles = 2;
				} break;
				
				// CPX abs
				case 0xEC: {
					cycles = 4;
					compare( CPU.X, read_abs() );
				} break;
				
				// SBC abs
				case 0xED: {
					cycles = 4;
					SBC( read_abs() );
				} break;
				
				// INC abs
				case 0xEE: {
					cycles = 6;
					INC( read_abs() );
				} break;
				
				//  BEQ rel
				case 0xF0: {
					cycles = 2;
					if ( CPU.read_flag(Z) )
						cycles += rel_jmp()? 2 : 1; 
				} break;
				
				// SBC ind,Y
				case 0xF1: {
					cycles = 5;
					SBC( read_ind_y(cycles) );
				} break;
				
				// SBC zpg,X
				case 0xF5: {
					cycles = 4;
					SBC( read_zpg_x() );
				} break;
				
				// INC zpg,X
				case 0xF6: {
					cycles = 6;
					INC( read_zpg_x() );
				} break;
				
				// SED impl
				case 0xF8: {
					cycles = 2;
					CPU.set_flag( D );
				} break;
				
				// SBC abs,Y
				case 0xF9: {
					cycles = 4;
					SBC( read_abs_y(cycles) );
				} break;
				
				// SBC abs,X
				case 0xFD: {
					cycles = 4;
					SBC( read_abs_x(cycles) );
				} break;
				
				// INC abs,X
				case 0xFE: {
					cycles = 6;
					u8 dummy;
					INC( read_abs_x(dummy) ); // TODO: clean up
				} break;
			}
			history.push( instruction, cycles, arg1, arg2 );
			do {
				auto cycle_length_us = (int)(1000000.0f / CPU.Hz);
				//usleep( cycle_length_us ); // TODO: find portable alternative
				std::this_thread::sleep_for( 1us * cycle_length_us );
				++CPU.cc;
			} while ( --cycles ); // TODO: verify
		}
	}
};



struct widget_i
{
	virtual ~widget_i() {}
	virtual void redraw() = 0;
	virtual void update() = 0;
};


/*
struct cpu_debug_15x10_display_t final: public widget_i
{
	cpu::cpu_t *CPU = nullptr;
	u8 x=0, y=0;
	
	virtual void
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
*/

struct cpu_debug_82x4_display_t final: public widget_i
{
private:
	cpu::cpu_t const *CPU = nullptr;
	u8 x=0, y=0;
	
public:
	cpu_debug_82x4_display_t() noexcept = default;
	
	cpu_debug_82x4_display_t( cpu::cpu_t const *CPU, u8 x, u8 y ) noexcept:
		widget_i(), CPU(CPU), x(x), y(y)
	{}
	
	~cpu_debug_82x4_display_t() noexcept final = default;
	
	void
	redraw() noexcept final
	{
	}
	
	void
	update() noexcept final
	{
		using namespace cpu;
		
		auto constexpr set    = "\e[1;32m1\e[0m";
		auto constexpr unset  = "\e[1;31m0\e[0m";
		
		assert( CPU );
		
		auto cc_digits = 1;
		auto tmp = CPU->cc;
		while ( tmp /= 10 )
			++cc_digits;
		
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



template <u8 width, u8 height>
struct terminal_display_t final: public widget_i
{
private:
	u8 const *block_start = nullptr;
	u8 x=0, y=0;
public:
	terminal_display_t() noexcept = default;
	
	terminal_display_t( u8 const *block_start, u8 x, u8 y ):
		block_start(block_start), x(x), y(y)
	{}
	
	~terminal_display_t() noexcept final = default;
	
	void
	redraw() noexcept final
	{
	}
	
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
};



template <u8 rows>
struct history_t final: public widget_i
{
private:
	system_t const *system = nullptr;
	u8              x      = 0,
	                y      = 0;
public:
	history_t() noexcept = default;
	
	history_t( system_t const *system, u8 x, u8 y ):
		widget_i(), system(system), x(x), y(y)
	{}
	
	~history_t() noexcept final = default;
	
	void
	redraw() noexcept final
	{
	}
	
	void
	update() noexcept
	{
		assert( system );
		
		std::printf( "\033[?25l\033[%u;%uH", y, x ); 
		
		u64 entries_to_print = system->history.size();
		if ( entries_to_print > rows )
			entries_to_print = rows;
		
		for ( u64 i=0; i<entries_to_print; ++i ) {
			auto const e = system->history.peek(i);
			printf( "$%04" PRIX16 "   ", e.addr );
			printf( system_t::op_meta_tbl.at(e.op).format, e.arg1, e.arg2 );
			printf( "   ; %" PRIu8 " cycles" "\033[%u;%uH", y+i, x, e.cycles );
		}
// std::printf( "\033[999;0H" LABEL "Last OP: " BRT_CLR "%-18s\033[0m", system_t::op_meta_tbl.at(system->history.peek().op).format ); // TODO: refactor
	}
};



struct ram_page_hex_display_t final: public widget_i
{
	// TODO: refactor and split into separate types:
	//       zpg_mem_display_t (page 0, show non-zero values as green, zero values as red)
	//       stk_mem_display_t (page 1, highlight all memory <= SP)
	//       prg_mem_display_t (focused on page of PC; highlight current instruction + args, otherwise red/green)
private:
	system_t const *system = nullptr;
	u8  page_no=0;
	u16 x=0, y=0;
	
public:
	ram_page_hex_display_t() noexcept = default;
	
	ram_page_hex_display_t( system_t const *system, u8 page_no, u16 x, u16 y ) noexcept:
		widget_i(), system(system), page_no(page_no), x(x), y(y)
	{}
	
	~ram_page_hex_display_t() noexcept final = default;
	
	void
	redraw() noexcept final
	{
	}
	
	void
	update() noexcept final
	{
#		define GOTO_NEXT_LINE   "\033[B\033[55D" 
		const u16 page_base_addr = page_no * 0x100;
		assert( system );
		
		std::printf( "\033[?25l\033[%u;%uH╭─────┰───────────────────────────────────────────────╮"
		             GOTO_NEXT_LINE "│" LABEL, y, x );
		if      ( page_no == 0 )
			std::printf( " ZPG " );
		else if ( page_no == 1 )
			std::printf( " STK " );
		else if ( page_no == 2 ) // TODO: change after proper display emulation is implemented
			std::printf( " DSP " );
		else if ( page_no == 3 ) // TODO: change after user is free to start programs anywhere
			std::printf( " PRG " );
		else
			std::printf( "Pg.%02" PRIX8, page_no );

		std::printf( BORDER "┃"    DIM_CLR "0"  BRT_CLR "0 " DIM_CLR "0"  BRT_CLR "1 "
			DIM_CLR "0"  BRT_CLR "2 " DIM_CLR "0"  BRT_CLR "3 " DIM_CLR "0"  BRT_CLR "4 " DIM_CLR "0"
			BRT_CLR "5 " DIM_CLR "0"  BRT_CLR "6 " DIM_CLR "0"  BRT_CLR "7 " DIM_CLR "0"  BRT_CLR "8 "
			DIM_CLR "0"  BRT_CLR "9 " DIM_CLR "0"  BRT_CLR "A " DIM_CLR "0"  BRT_CLR "B " DIM_CLR "0"
			BRT_CLR "C " DIM_CLR "0"  BRT_CLR "D " DIM_CLR "0"  BRT_CLR "E " DIM_CLR "0"  BRT_CLR "F"
			BORDER "│" GOTO_NEXT_LINE
			"┝━━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┥" GOTO_NEXT_LINE
		);
		u16 const stack_end = 0x0100 + system->CPU.SP;
		u16 curr_addr = page_base_addr;
		for ( u8 row=0; row<0x10; ++row ) {
			std::printf( "│" DIM_CLR "%02" PRIX8 BRT_CLR "%1" PRIX8 DIM_CLR "0" BORDER " ┃", page_no, row );
			for ( u8 col=0; col<0x10; ++col ) {
				auto const curr_byte = system->RAM[curr_addr++];
				if ( page_no == 1 ) // stack
					std::printf( "\e[%sm", curr_addr <= stack_end? "1;97":"90" );
				else 
					std::printf( "\e[1;%sm", curr_addr==system->CPU.PC? "1;40;93" : curr_byte? "32":"31" );
				std::printf( "%02" PRIX8 "\e[0m%s", curr_byte, col==0xF?"":" " );
			}
			std::printf( "\e[0m│" GOTO_NEXT_LINE );
		}
		std::printf("╰─────┸───────────────────────────────────────────────╯");
#		undef GOTO_NEXT_LINE
	}
};



int
main( int const argc, char const *const argv[] )
{
	auto system        = system_t {};
	u8 *display_block  = system.RAM+0x0200;
	auto terminal      = terminal_display_t<80,25> { display_block, 2,1  };
	//auto dbg_cpu_old = cpu_debug_15x10_display_t { &system.CPU, 124,0  };
	auto dbg_cpu       = cpu_debug_82x4_display_t  { &system.CPU, 0,27 };
	auto history       = history_t<16>             { &system, 0, 33 };
	u8 constexpr pg_x=4, pg_y=4;
	widget_i *widgets[pg_x * pg_y + 3] {};
	
	widgets[0] = (widget_i*)&terminal;
	widgets[1] = (widget_i*)&dbg_cpu;
	widgets[3] = (widget_i*)&history;
	u8 pg_no = 0;
	for ( u8 y=0; y<pg_y; ++y )
		for ( u8 x=0; x<pg_x; ++x )
			widgets[3 + y*pg_x+x] = new ram_page_hex_display_t { &system, pg_no++, u16(x*56U+85U), u16(y*20U+1U) };
	
	system.CPU.Hz = argc > 1? std::atoi(argv[1]) : 500;
	auto txt      = " xx Hello world!";
	std::memcpy( (void*)display_block, txt, 16 );
	
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
	
	auto cpu_thread = std::jthread(
		[&system] {
			system.update();
		}
	);

	auto const target_framerate = (argc>2? std::atoi(argv[2]) : 30);
	auto const frame_length_ms  = 1000ms / target_framerate;
	auto gui_thread = std::jthread(
		[&widgets, &frame_length_ms] {
			while (1) {
				if ( false ) // TODO
					for ( auto *w: widgets )
						w->redraw();
				for ( auto *w: widgets )
					w->update();
			std::this_thread::sleep_for(frame_length_ms);
			}
		}
	);
	
	/*
	
	f32 constexpr draw_frame_window_ms = 1000.0f / 30;
	
	f32 time_since_draw_ms = draw_frame_window_ms,
	    frame_time_elapsed = .0f;
	
	auto user_io_thread = std::jthread(
		[&keyboard,&mouse] {
			keyboard.update();
			mouse.update();
		}
	);
	*/
	/*
	while (true) {
		frame_time_elapsed = timer();
		
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
	*/
	
	while (1); // TODO: remove
	
	for ( auto *w: widgets )
		delete w; // TODO: get rid of later with RAAI
}

