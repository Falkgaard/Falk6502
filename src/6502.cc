#include <cstdint>
#include <cinttypes>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>

#include <string>
#include <chrono>
#include <thread>
#include <utility>
#include <unordered_map>

namespace curses {
	#include "curses.h" // TODO: statically link in the make file
}

struct tui_app_t final
{
	tui_app_t()
	{
		// TODO: create in main
		curses::initscr();
		curses::cbreak();
		curses::noecho();
		curses::keypad( curses::stdscr, true );
		// keyboard IO thread
		// widgets  IO thread
	}
	
	~tui_app_t() noexcept
	{
		curses::endwin();
	}
private:
};

using namespace::std::literals;

// TODO: suffix with CLR?
#define LABEL         "\033[1;33m"
#define OUTER_BORDER  "\033[0;1;97m"
#define INNER_BORDER  "\033[1;97m"
#define DIM_CLR       "\033[90m"
#define BRT_CLR       "\033[97m"

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
		assert( port and is_running_ptr );
		*port = getchar();
		// terminate when "x" is pressed
		if ( *port == 'x' )
			*is_running_ptr = false;
		else if ( *port == 's' )
			*is_stepping_ptr = not *is_stepping_ptr; // flip
		else if ( *is_stepping_ptr and *port == 'n' )
			*should_step_ptr = true;
	}
	u8   *port            = nullptr;
	bool *is_running_ptr  = nullptr;
	bool *is_stepping_ptr = nullptr;
	bool *should_step_ptr = nullptr;
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
		u32 last_write_addr = -1; // too large to allow for out-of-bounds values (-1)
		u16 last_push_addr  = -1; // too large to allow for out-of-bounds values (-1)
		u16 last_addr       =  0; // not really needed here (since every cycle will set it)

		
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

template <typename T>
struct cyclic_stack_t final
{
	cyclic_stack_t( u64 const capacity ) noexcept:
		_data     ( new T[capacity] ),
		_capacity ( capacity )
	{}
	
	~cyclic_stack_t() noexcept {
		delete[] _data;
	}
	
	cyclic_stack_t( cyclic_stack_t const &other ) noexcept
	{
		*this = other;
	}

	cyclic_stack_t( cyclic_stack_t &&other ) noexcept
	{
		*this = other;
	}
	
	auto
	operator=( cyclic_stack_t const &other ) noexcept -> cyclic_stack_t&
	{
		if ( &other != this )
		{
			if ( _data )
				delete[] _data;
			_capacity = other._capacity;
			_size     = other._size;
			_head     = other._head;
			_data     = new T[_capacity];
			// deep copy:
			for ( u64 tail=(_capacity+_head-_size)%_capacity, i=0; i<_size; ++i ) {
				u64 idx = (tail+i) % _capacity;
				_data[idx] = other._data[idx];
			}
		}
	}
	
	auto
	operator=( cyclic_stack_t &&other ) noexcept -> cyclic_stack_t&
	{
		if ( _data )
			delete[] _data;
		_capacity = other._capacity;
		_head     = other._head;
		_size     = other._size;
		_data     = std::exchange( other._data, nullptr );
	}
	
	template <typename...Args>
	inline void
	push( Args &&...args ) noexcept
	{
		_data[_head] = { std::forward<Args>(args)... };
		_head        = (_head+1) % _capacity;
		if ( _size < _capacity )
			++_size;
	}
	
	[[nodiscard]] inline auto
	pop() noexcept -> T
	{
		assert( _size and "Can't pop() empty stack!" );
		_head = (_capacity+_head-1) % _capacity;
		--_size;
		return _data[_head];
	}
	
	[[nodiscard]] inline auto
	peek( u64 const offset=0 ) const noexcept -> const T&
	{
		assert( offset < _size and "Trying to peek() out of bounds!" );
		return _data[(_capacity+_head-offset-1) % _capacity];
	}
	
	[[nodiscard]] inline auto
	size() const noexcept -> u64
	{
		return _size;
	}
	
private:
	T   *_data     = nullptr;
	u64  _capacity = 0;
	u64  _size     = 0;
	u16  _head     = 0;
};

struct history_data_t {
	u16 addr=0x0000;
	u8  op=0xEA, cycles=0, arg1=0, arg2=0;
};

using history_stack_t = cyclic_stack_t<history_data_t>;

enum address_mode_e: u8 {
	implied,
	accumulator,
	immediate,
	absolute,
	absolute_x,
	absolute_y,
	zero_page,
	zero_page_x,
	zero_page_y,
	indirect,
	indirect_x,
	indirect_y,
	relative
};

[[nodiscard]] auto constexpr
get_address_mode_size( address_mode_e const mode ) noexcept -> u8
{
	switch (mode) {
		case implied:     return 1;
		case accumulator: return 1;
		case immediate:   return 2;
		case absolute:    return 3;
		case absolute_x:  return 3;
		case absolute_y:  return 3;
		case zero_page:   return 2;
		case zero_page_x: return 2;
		case zero_page_y: return 2;
		case indirect:    return 3;
		case indirect_x:  return 2;
		case indirect_y:  return 2;
		case relative:    return 2;
	}
	assert( false and "Invalid address mode!" );
	return 0;
}

struct op_info_t {
	char const     *mnemonic;
	address_mode_e  address_mode;
};

// TODO: replace with a ADS that isn't absolute shit
const inline auto
op_meta_tbl = std::unordered_map<u8,op_info_t>
{
//   key    ________value_________
//   code  { mnemonic addressing  }
	{ 0x00, { "BRK",   implied     } },
	{ 0x01, { "ORA",   indirect_x  } },
	{ 0x05, { "ORA",   zero_page   } },
	{ 0x06, { "ASL",   zero_page   } },
	{ 0x08, { "PHP",   implied     } },
	{ 0x09, { "ORA",   immediate   } },
	{ 0x0A, { "ASL",   accumulator } },
	{ 0x0D, { "ORA",   absolute    } },
	{ 0x0E, { "ASL",   absolute    } },
	{ 0x10, { "BPL",   relative    } },
	{ 0x11, { "ORA",   indirect_y  } },
	{ 0x15, { "ORA",   zero_page_x } },
	{ 0x16, { "ASL",   zero_page_x } },
	{ 0x18, { "CLC",   implied     } },
	{ 0x19, { "ORA",   absolute_y  } },
	{ 0x1D, { "ORA",   absolute_x  } },
	{ 0x1E, { "ASL",   absolute_x  } },
	{ 0x20, { "JSR",   absolute    } },
	{ 0x21, { "AND",   indirect_x  } },
	{ 0x24, { "BIT",   zero_page   } },
	{ 0x25, { "AND",   zero_page   } },
	{ 0x26, { "ROL",   zero_page   } },
	{ 0x28, { "PLP",   implied     } },
	{ 0x29, { "AND",   immediate   } },
	{ 0x2A, { "ROL",   accumulator } },
	{ 0x2C, { "BIT",   absolute    } },
	{ 0x2D, { "AND",   absolute    } },
	{ 0x2E, { "ROL",   absolute    } },
	{ 0x30, { "BMI",   relative    } },
	{ 0x31, { "AND",   indirect_y  } },
	{ 0x35, { "AND",   zero_page_x } },
	{ 0x36, { "ROL",   zero_page_x } },
	{ 0x38, { "SEC",   implied     } },
	{ 0x39, { "AND",   absolute_y  } },
	{ 0x3D, { "AND",   absolute_x  } },
	{ 0x3E, { "ROL",   absolute_x  } },
	{ 0x40, { "RTI",   implied     } },
	{ 0x41, { "EOR",   indirect_x  } },
	{ 0x45, { "EOR",   zero_page   } },
	{ 0x46, { "LSR",   zero_page   } },
	{ 0x48, { "PHA",   implied     } },
	{ 0x49, { "EOR",   immediate   } },
	{ 0x4A, { "LSR",   accumulator } },
	{ 0x4C, { "JMP",   absolute    } },
	{ 0x4D, { "EOR",   absolute    } },
	{ 0x4E, { "LSR",   absolute    } },
	{ 0x50, { "BVC",   relative    } },
	{ 0x51, { "EOR",   indirect_y  } },
	{ 0x55, { "EOR",   zero_page_x } },
	{ 0x56, { "LSR",   zero_page_x } },
	{ 0x58, { "CLI",   implied     } },
	{ 0x59, { "EOR",   absolute_y  } },
	{ 0x5D, { "EOR",   absolute_x  } },
	{ 0x5E, { "LSR",   absolute_x  } },
	{ 0x60, { "RTS",   implied     } },
	{ 0x61, { "ADC",   indirect_x  } },
	{ 0x65, { "ADC",   zero_page   } },
	{ 0x66, { "ROR",   zero_page   } },
	{ 0x68, { "PLA",   implied     } },
	{ 0x69, { "ADC",   immediate   } },
	{ 0x6A, { "ROR",   accumulator } },
	{ 0x6C, { "JMP",   indirect    } },
	{ 0x6D, { "ADC",   absolute    } },
	{ 0x6E, { "ROR",   absolute    } },
	{ 0x70, { "BVS",   relative    } },
	{ 0x71, { "ADC",   indirect_y  } },
	{ 0x75, { "ADC",   zero_page_x } },
	{ 0x76, { "ROR",   zero_page_x } },
	{ 0x78, { "SEI",   implied     } },
	{ 0x79, { "ADC",   absolute_y  } },
	{ 0x7D, { "ADC",   absolute_x  } },
	{ 0x7E, { "ROR",   absolute_x  } },
	{ 0x81, { "STA",   indirect_x  } },
	{ 0x84, { "STY",   zero_page   } },
	{ 0x85, { "STA",   zero_page   } },
	{ 0x86, { "STX",   zero_page   } },
	{ 0x88, { "DEY",   implied     } },
	{ 0x8A, { "TXA",   implied     } },
	{ 0x8C, { "STY",   absolute    } },
	{ 0x8D, { "STA",   absolute    } },
	{ 0x8E, { "STX",   absolute    } },
	{ 0x90, { "BCC",   relative    } },
	{ 0x91, { "STA",   indirect_y  } },
	{ 0x94, { "STY",   zero_page_x } },
	{ 0x95, { "STA",   zero_page_x } },
	{ 0x96, { "STX",   zero_page_y } },
	{ 0x98, { "TYA",   implied     } },
	{ 0x99, { "STA",   absolute_y  } },
	{ 0x9A, { "TXS",   implied     } },
	{ 0x9D, { "STA",   absolute_x  } },
	{ 0xA0, { "LDY",   immediate   } },
	{ 0xA1, { "LDA",   indirect_x  } },
	{ 0xA2, { "LDX",   immediate   } },
	{ 0xA4, { "LDY",   zero_page   } },
	{ 0xA5, { "LDA",   zero_page   } },
	{ 0xA6, { "LDX",   zero_page   } },
	{ 0xA8, { "TAY",   implied     } },
	{ 0xA9, { "LDA",   immediate   } },
	{ 0xAA, { "TAX",   implied     } },
	{ 0xAC, { "LDY",   absolute    } },
	{ 0xAD, { "LDA",   absolute    } },
	{ 0xAE, { "LDX",   absolute    } },
	{ 0xB0, { "BCS",   relative    } },
	{ 0xB1, { "LDA",   indirect_y  } },
	{ 0xB4, { "LDY",   zero_page_x } },
	{ 0xB5, { "LDA",   zero_page_x } },
	{ 0xB6, { "LDX",   zero_page_y } },
	{ 0xB8, { "CLV",   implied     } },
	{ 0xB9, { "LDA",   absolute_y  } },
	{ 0xBA, { "TSX",   implied     } },
	{ 0xBC, { "LDY",   absolute_x  } },
	{ 0xBD, { "LDA",   absolute_x  } },
	{ 0xBE, { "LDX",   absolute_y  } },
	{ 0xC0, { "CPY",   immediate   } },
	{ 0xC1, { "CMP",   indirect_x  } },
	{ 0xC4, { "CPY",   zero_page   } },
	{ 0xC5, { "CMP",   zero_page   } },
	{ 0xC6, { "DEC",   zero_page   } },
	{ 0xC8, { "INY",   implied     } },
	{ 0xC9, { "CMP",   immediate   } },
	{ 0xCA, { "DEX",   implied     } },
	{ 0xCC, { "CPY",   absolute    } },
	{ 0xCD, { "CMP",   absolute    } },
	{ 0xCE, { "DEC",   absolute    } },
	{ 0xD0, { "BNE",   relative    } },
	{ 0xD1, { "CMP",   indirect_y  } },
	{ 0xD5, { "CMP",   zero_page_x } },
	{ 0xD6, { "DEC",   zero_page_x } },
	{ 0xD8, { "CLD",   implied     } },
	{ 0xD9, { "CMP",   absolute_y  } },
	{ 0xDD, { "CMP",   absolute_x  } },
	{ 0xDE, { "DEC",   absolute_x  } },
	{ 0xE0, { "CPX",   immediate   } },
	{ 0xE1, { "SBC",   indirect_x  } },
	{ 0xE4, { "CPX",   zero_page   } },
	{ 0xE5, { "SBC",   zero_page   } },
	{ 0xE6, { "INC",   zero_page   } },
	{ 0xE8, { "INX",   implied     } },
	{ 0xE9, { "SBC",   immediate   } },
	{ 0xEA, { "NOP",   implied     } },
	{ 0xEC, { "CPX",   absolute    } },
	{ 0xED, { "SBC",   absolute    } },
	{ 0xEE, { "INC",   absolute    } },
	{ 0xF0, { "BEQ",   relative    } },
	{ 0xF1, { "SBC",   indirect_y  } },
	{ 0xF5, { "SBC",   zero_page_x } },
	{ 0xF6, { "INC",   zero_page_x } },
	{ 0xF8, { "SED",   implied     } },
	{ 0xF9, { "SBC",   absolute_y  } },
	{ 0xFD, { "SBC",   absolute_x  } },
	{ 0xFE, { "INC",   absolute_x  } }
};

[[nodiscard]] auto
get_op_byte_size( u8 const op ) noexcept -> u8
{
	// TODO: handle better?
	try {
		return get_address_mode_size( op_meta_tbl.at(op).address_mode );
	}
	catch(...) {
		assert( false and "Invalid address mode!" );
		return 0;
	}
}



struct system_t
{
private:
	cpu::cpu_t        CPU     = {}; // MOS Technology 6502
	u8               *RAM     = nullptr;
	history_stack_t   history = { 128 };
	
public:
	system_t() noexcept:
		RAM( new u8[0x10000] )
	{}
	
	[[nodiscard]] inline auto
	get_cpu() noexcept -> cpu::cpu_t&
	{
		return CPU;
	}
	
	[[nodiscard]] inline auto
	get_ram() noexcept -> u8*
	{
		return RAM;
	}
	
	[[nodiscard]] inline auto
	get_history() noexcept -> history_stack_t&
	{
		return history;
	}
	
	[[nodiscard]] inline auto
	get_cpu() const noexcept -> cpu::cpu_t const&
	{
		return CPU;
	}
	
	[[nodiscard]] inline auto
	get_ram() const noexcept -> u8 const*
	{
		return RAM;
	}
	
	[[nodiscard]] inline auto
	get_history() const noexcept -> history_stack_t const&
	{
		return history;
	}
	
	static inline void
	print_asm( u8 const op_code,
	           u8 const arg1=0,
	           u8 const arg2=0 )
	{
#define SYM_CLR "\033[0;1;33m"
#define IMM_CLR "\033[0;1;31m"
#define ZPG_CLR "\033[0;1;37m"
#define HEX_CLR "\033[0;1;93m"
#define NUM_CLR "\033[0;1;97m"
#define REG_CLR "\033[0;1;36m"
		//std::fprintf( stderr, "at(op_code: %02" PRIX8 ")\n", op_code );
		try {
			auto const info = op_meta_tbl.at(op_code); // TODO: handle exception?
			std::printf( "\033[0;1;33m%.3s\033[0m ", info.mnemonic ); // print instruction
			switch ( info.address_mode ) {
				case implied:     std::printf(         " "         " "         "  "                " "           " "           " "         " "             ); break;
				case accumulator: std::printf( REG_CLR " "         "A"         "  "                " "           " "           " "         " "             ); break;
				case immediate:   std::printf( IMM_CLR "#" HEX_CLR "$" NUM_CLR "%02" PRIX8         " "           " "           " "         " ", arg1       ); break;
				case absolute:    std::printf(         " " HEX_CLR "$" NUM_CLR "%02" PRIX8         "%02"         PRIX8         " "         " ", arg1, arg2 ); break;
				case absolute_x:  std::printf(         " " HEX_CLR "$" NUM_CLR "%02" PRIX8         "%02"         PRIX8 SYM_CLR "," REG_CLR "X", arg1, arg2 ); break;
				case absolute_y:  std::printf(         " " HEX_CLR "$" NUM_CLR "%02" PRIX8         "%02"         PRIX8 SYM_CLR "," REG_CLR "Y", arg1, arg2 ); break;
				case zero_page:   std::printf( ZPG_CLR "*" HEX_CLR "$" NUM_CLR "%02" PRIX8         " "           " "           " "         " ", arg1       ); break;
				case zero_page_x: std::printf( ZPG_CLR "*" HEX_CLR "$" NUM_CLR "%02" PRIX8 SYM_CLR ","   REG_CLR "X"           " "         " ", arg1       ); break;
				case zero_page_y: std::printf( ZPG_CLR "*" HEX_CLR "$" NUM_CLR "%02" PRIX8 SYM_CLR ","   REG_CLR "Y"           " "         " ", arg1       ); break;
				case indirect:    std::printf( SYM_CLR "(" HEX_CLR "$" NUM_CLR "%02" PRIX8         "%02"         PRIX8 SYM_CLR ")"         " ", arg1, arg2 ); break;
				case indirect_x:  std::printf( SYM_CLR "(" HEX_CLR "$" NUM_CLR "%02" PRIX8 SYM_CLR ","   REG_CLR "X"   SYM_CLR ")"         " ", arg1       ); break;
				case indirect_y:  std::printf( SYM_CLR "(" HEX_CLR "$" NUM_CLR "%02" PRIX8 SYM_CLR ")"           ","   REG_CLR "Y"         " ", arg1       ); break;
				case relative:    std::printf(         " " HEX_CLR "$" NUM_CLR "%02" PRIX8         " "           " "           " "         " ", arg1       ); break;
			}
		}
		catch(...) {
			std::printf("\033[0;1;31mERR INVALID " );
		}
#undef SYM_CLR
#undef IMM_CLR
#undef ZPG_CLR
#undef HEX_CLR
#undef REG_CLR
	}
	
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
		
		// TODO: fix cycle elapse pre OP side-effects instead of post
		CPU.last_addr        = CPU.PC;
		u8 const instruction = RAM[CPU.PC++];
		u8 const arg1        = RAM[CPU.PC  ];
		u8 const arg2        = RAM[CPU.PC+1];
		
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
			
			default: {
				assert( false and "Invalid OP code!" );
				cycles = 0;
				// TODO: interrupt?
			}
		}
		history.push( CPU.last_addr, instruction, cycles, arg1, arg2 );
		if ( cycles == 0 )
			std::this_thread::sleep_for( 10s ); // TEMP
		do {
			auto cycle_length_us = (int)(1000000.0f / CPU.Hz);
			std::this_thread::sleep_for( 1us * cycle_length_us );
			++CPU.cc;
		} while ( --cycles ); // TODO: verify
	}
};



struct widget_i
{
	virtual ~widget_i() {}
	virtual void redraw() = 0;
	virtual void update() = 0;
};


/* DEPRECATED
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
		
		auto constexpr set    = "\033[1;32m1\033[0m";
		auto constexpr unset  = "\033[1;31m0\033[0m";
		
		assert( CPU );
		
		auto cc_digits = 1;
		auto tmp = CPU->cc;
		while ( tmp /= 10 )
			++cc_digits;
		
		std::printf( "\033[?25l\033[%u;%uH", y, x ); 
		std::printf( 
			"╭────┰────────────────┰──────┰────┰────┰────┰────┰──────────┰──────────────────────╮"  "\033[B\033[84D"
			"│ " LABEL "F:" OUTER_BORDER " ┃ " LABEL "N V BB D I Z C" OUTER_BORDER " ┃ " LABEL "PC:" OUTER_BORDER
			"  ┃ " LABEL "S:" OUTER_BORDER " ┃ " LABEL "A:" OUTER_BORDER " ┃ " LABEL "X:" OUTER_BORDER " ┃ " LABEL "Y:" OUTER_BORDER
			" ┃ " LABEL "Clk.Spd:" OUTER_BORDER " ┃ " LABEL "Elapsed Cycle Count:" OUTER_BORDER " │"  "\033[B\033[84D"
			"│ " BRT_CLR "%02" PRIX8 OUTER_BORDER " ┃ %s %s %s%s %s %s %s %s ┃ " BRT_CLR "%04" PRIX16 OUTER_BORDER " ┃ "
			BRT_CLR "%02" PRIX8 OUTER_BORDER " ┃ " BRT_CLR "%02" PRIX8 OUTER_BORDER
			" ┃ " BRT_CLR "%02" PRIX8 OUTER_BORDER " ┃ " BRT_CLR "%02" PRIX8 OUTER_BORDER " ┃ " BRT_CLR "%5" PRIu16 OUTER_BORDER " Hz" 
			" ┃ " DIM_CLR "%0*d" BRT_CLR "%" PRIu64 OUTER_BORDER " │"                   "\033[B\033[84D"
			"╰────┸────────────────┸──────┸────┸────┸────┸────┸──────────┸──────────────────────╯",
			CPU->P,
			CPU->read_flag(N)?set:unset, CPU->read_flag(V)?set:unset, CPU->read_flag(BU)?set:unset, CPU->read_flag(BL)?set:unset,
			CPU->read_flag(D)?set:unset, CPU->read_flag(I)?set:unset, CPU->read_flag( Z)?set:unset, CPU->read_flag( C)?set:unset,
			CPU->PC, CPU->SP, CPU->A, CPU->X, CPU->Y, CPU->Hz, 20-cc_digits, 0, CPU->cc
		);
	}
private:
	cpu::cpu_t const *CPU = nullptr;
	u8 x=0, y=0;
};



template <u8 width, u8 height>
struct terminal_display_t final: public widget_i
{
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
private:
	u8 const *block_start = nullptr;
	u8        x=0, y=0;
};



template <u8 rows>
struct history_log_t final: public widget_i
{
	history_log_t() noexcept = default;
	
	history_log_t( system_t const *system, u8 x, u8 y ):
		widget_i(), system(system), x(x), y(y)
	{}
	
	~history_log_t() noexcept final = default;
	
	void
	redraw() noexcept final
	{
	}
	
	void
	update() noexcept
	{
		assert( system );
		
		std::printf( "\033[?25l\033[%u;%uH" OUTER_BORDER "╭───────────────────────────╮" "\033[B\033[%uG", y, x, x ); 
		
		u8 entries_to_print = system->get_history().size();
		if ( entries_to_print > rows )
			entries_to_print = rows;
		
		for ( u8 i=0; i<entries_to_print; ++i ) {
			auto const e    = system->get_history().peek(i);
			std::printf( OUTER_BORDER "│" "\033[0;38m" "%04" PRIX16 " ", e.addr );
			system_t::print_asm( e.op, e.arg1, e.arg2 );
			std::printf( "\033[0;38;5;238m" " ;" "\033[0;90m%" PRIu8 " cycles" OUTER_BORDER "│" "\033[B\033[%uG", e.cycles, x );
		}
		for ( u8 i=rows-entries_to_print; i; --i )
			std::printf( OUTER_BORDER "│                           │" "\033[B\033[%uG", x );
		std::printf(    OUTER_BORDER "╰───────────────────────────╯" );
	}
private:
	system_t const *system = nullptr;
	u8              x      = 0,
	                y      = 0;
};



struct stack_hex_display_t final: public widget_i
{
	stack_hex_display_t() noexcept = default;
	
	stack_hex_display_t( system_t const *system, u16 x, u16 y ) noexcept:
		widget_i(), system(system), x(x), y(y)
	{}
	
	~stack_hex_display_t() noexcept final = default;
	
	void
	redraw() noexcept final
	{
	}
	
	void
	update() noexcept final
	{
#		define GOTO_NEXT_LINE   "\033[B\033[54D" 
		const u16 page_base_addr = 0x0100;
		assert( system );
		auto const &CPU = system->get_cpu();
		auto const *RAM = system->get_ram();
		u16  const stack_end = 0x0100 + CPU.SP;
		
		std::printf( "\033[?25l\033[%u;%uH" OUTER_BORDER "╭────┰───────────────────────────────────────────────╮"
		             GOTO_NEXT_LINE "│" LABEL " STK", y, x );
		
		std::printf( INNER_BORDER "┃" );
		for ( u8 col=0; col<0x10; ++col )
			std::printf( DIM_CLR "0"  "%s%" PRIX8 "%s", (page_base_addr+col == (CPU.PC&0xFF0F)? LABEL : BRT_CLR), col, (col==0xF? "":" ") );
		std::printf( OUTER_BORDER "│" GOTO_NEXT_LINE "┝" INNER_BORDER "━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" OUTER_BORDER "┥" GOTO_NEXT_LINE );
		u16 curr_addr = page_base_addr;
		for ( u8 row=0; row<0x10; ++row ) {
			std::printf( OUTER_BORDER "│" "\033[90m" "01" BRT_CLR "%1" PRIX8 DIM_CLR "0" INNER_BORDER "┃", row );
			for ( u8 col=0; col<0x10; ++col, ++curr_addr ) {
				auto const curr_byte = RAM[curr_addr];
				std::printf( "\033[%sm", curr_addr < stack_end? "1;97":"90" );
				std::printf( "%02" PRIX8 "%s", curr_byte, col==0xF?"":" " );
			}
			std::printf( OUTER_BORDER "│" GOTO_NEXT_LINE );
		}
		std::printf("╰────┸───────────────────────────────────────────────╯");
#		undef GOTO_NEXT_LINE
	}
private:
	system_t const *system = nullptr;
	u16 x=0, y=0;
};



struct ram_page_hex_display_t final: public widget_i
{
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
#		define GOTO_NEXT_LINE   "\033[B\033[54D" 
		const u16 page_base_addr = page_no * 0x100;
		assert( system );
		
		std::printf( "\033[?25l\033[%u;%uH" OUTER_BORDER "╭────┰───────────────────────────────────────────────╮"
		             GOTO_NEXT_LINE "│" LABEL, y, x );
		if      ( page_no == 0 )
			std::printf( " ZPG" );
		else if ( page_no == 1 )
			std::printf( " STK" );
		else
			std::printf( "pg%02" PRIX8, page_no );
		
		auto const &CPU = system->get_cpu();
		auto const *RAM = system->get_ram();
		
		std::printf( INNER_BORDER "┃" );
		for ( u8 col=0; col<0x10; ++col )
			std::printf( DIM_CLR "0" BRT_CLR "%" PRIX8 "%s", col, (col==0xF? "":" ") );
		std::printf( OUTER_BORDER "│" GOTO_NEXT_LINE "┝" INNER_BORDER "━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" OUTER_BORDER "┥" GOTO_NEXT_LINE );
		u16 curr_addr = page_base_addr;
		for ( u8 row=0; row<0x10; ++row ) {
			std::printf( OUTER_BORDER "│" "\033[90m%02" PRIX8 "%s%1" PRIX8 DIM_CLR "0" INNER_BORDER "┃",
			             page_no, BRT_CLR, row );
			for ( u8 col=0; col<0x10; ++col, ++curr_addr ) {
				auto const curr_byte = RAM[curr_addr];
				std::printf( "\033[1;%sm", curr_byte? "1;97":"90" );
				std::printf( "%02" PRIX8 "%s", curr_byte, col==0xF?"":" " );
			}
			std::printf( OUTER_BORDER "│" GOTO_NEXT_LINE );
		}
		std::printf("╰────┸───────────────────────────────────────────────╯");
#		undef GOTO_NEXT_LINE
	}
private:
	system_t const *system = nullptr;
	u8  page_no=0;
	u16 x=0, y=0;
};

// TODO: proper hex+ascii editor 

struct program_hex_display_t final: public widget_i
{
	program_hex_display_t () noexcept = default;
	
	program_hex_display_t ( system_t const *system, u16 x, u16 y ) noexcept:
		widget_i(), system(system), x(x), y(y)
	{}
	
	~program_hex_display_t () noexcept final = default;
	
	void
	redraw() noexcept final
	{
	}
	
	void
	update() noexcept final
	{
#		define GOTO_NEXT_LINE   "\033[B\033[54D" 
#		define CROSS_CLR        "\033[48;5;237m"
		assert( system );
		auto const &CPU     = system->get_cpu();
		auto const *RAM     = system->get_ram();
		auto const  PC      = CPU.last_addr;
		auto const  OP      = RAM[PC];
		u16  const  page_no = PC >> 8;
		
		std::printf( "\033[?25l\033[%u;%uH" OUTER_BORDER "╭────┰───────────────────────────────────────────────╮"
		             GOTO_NEXT_LINE "│" LABEL "Pg" BRT_CLR "%02" PRIX8, y, x, page_no );
		
		std::printf( INNER_BORDER "┃" );
		for ( u8 col=0; col<0x10; ++col ) {
			auto const is_active_col = col == (PC&0xF);
			std::printf( "%s" DIM_CLR "0"  "%s%" PRIX8 "\033[0m%s", (is_active_col? CROSS_CLR : ""), (is_active_col? LABEL : BRT_CLR), col, (col==0xF? "":" ") );
		}
		
		std::printf( OUTER_BORDER "│" GOTO_NEXT_LINE "┝" INNER_BORDER "━━━━╋" );
		
		for ( u8 col=0; col<0x10; ++col )
			std::printf( "%s", col == (PC&0xF)? CROSS_CLR "━━\033[0m" INNER_BORDER "━" : "━━━" );
		
		std::printf( OUTER_BORDER "\033[D┥" GOTO_NEXT_LINE );
		
		auto const op_start = PC;
		auto const op_end   = PC + get_op_byte_size(OP) - 1;
			
		u16 curr_addr = PC & 0xFF00;
		for ( u8 row=0; row<0x10; ++row ) {
			bool is_active_row = (curr_addr&0xFFF0)==(PC&0xFFF0);
			std::printf( OUTER_BORDER "│" "%s" "%02" PRIX8 "%s%1" PRIX8 DIM_CLR "0" INNER_BORDER "┃",
			             (is_active_row? CROSS_CLR "\033[1;33m" : "\033[90m"), page_no, (is_active_row? LABEL : BRT_CLR), row );
			for ( u8 col=0; col<0x10; ++col, ++curr_addr ) {
				bool is_active_col = (curr_addr&0xFF0F) == (PC&0xFF0F);
				auto const curr_byte = RAM[curr_addr];
				if ( curr_addr < op_start or curr_addr > op_end )
					std::printf( "\033[%sm", curr_byte? "32" : "31" ); // green (>0) or red (00)
				else if ( curr_addr == op_start ) // start of current op highlighting
					std::printf("\033[1;93;48;5;235m");
				else // arg1 or arg2
					std::printf("\033[1;97m");
				if ( not is_active_row )
					std::printf( "%s" "%02" PRIX8 "\033[0m%s", (is_active_col? CROSS_CLR : ""), curr_byte, col==0xF?"":" " );
				else {
					std::printf( "%02" PRIX8 CROSS_CLR "%s", curr_byte, col==0xF?"":" " );
				}
			}
			std::printf( OUTER_BORDER "│" GOTO_NEXT_LINE );
		}
		std::printf("╰────┸───────────────────────────────────────────────╯");
#		undef GOTO_NEXT_LINE
	}
private:
	system_t const *system = nullptr;
	u16 x=0, y=0;
};

struct log_entry_t
{
	log_entry_t() noexcept = default;
	
	log_entry_t( char const *message )
	{
		_create_entry(message);
	}
	
	inline auto
	operator=( char const *message ) noexcept -> log_entry_t&
	{
		_create_entry(message);
		return *this;
	}
	
	[[nodiscard]] inline auto
	timestamp() const noexcept -> char const*
	{
		return _timestamp;
	}
	
	[[nodiscard]] inline auto
	msg() const noexcept -> char const*
	{
		return _msg;
	}
	
	// TODO
	void save_to_disk( char const *filename ) noexcept
	{
	}
	
private:
	char _timestamp[  9] = "";
	char       _msg[128] = "";
	
	inline void
	_create_entry( char const *message ) noexcept
	{
		auto const  t  =  std::time(nullptr);
		auto const &lt = *std::localtime(&t);
		std::sprintf(
			_timestamp,
			"%.2d:%.2d:%.2d",
			//1900+lt.tm_year, lt.tm_mon, lt.tm_mday, // just hh:mm:ss should suffice
			lt.tm_hour, lt.tm_min, lt.tm_sec
		);
		std::sprintf( _msg, "%-.127s", message );
	}
};

using log_stack_t = cyclic_stack_t<log_entry_t>;

template <u8 rows>
struct log_widget_t final: public widget_i
{
	log_widget_t () noexcept = default;
	
	log_widget_t ( u16 x, u16 y ) noexcept:
		widget_i(), entries(rows), x(x), y(y)
	{}
	
	~log_widget_t () noexcept final = default;
	
	inline void
	push( char const *msg ) noexcept
	{
		entries.push(msg);	
	}
	
	void
	redraw() noexcept final
	{
	}
	
	void
	update() noexcept final
	{
		std::printf( "\033[?25l\033[%u;%uH"
			             OUTER_BORDER "╭────────────────────────────────────────────────────╮" "\033[B\033[%uG", y, x, x ); 
			
		u8 entries_to_print = entries.size();
		if ( entries_to_print > rows )
			entries_to_print = rows;
		
		for ( u8 i=0; i<entries_to_print; ++i ) {
			auto const &e = entries.peek(i);
			std::printf( OUTER_BORDER "│ \033[1;33m%s" " \033[37m%-.39s", e.timestamp(), e.msg() );
			std::printf( "\033[%uG" OUTER_BORDER " │" "\033[B\033[%uG", x+52, x );
		}
		
		for ( u8 i=rows-entries_to_print; i; --i )
			std::printf( OUTER_BORDER "│                                                    │" "\033[B\033[%uG", x );
		std::printf(    OUTER_BORDER "╰────────────────────────────────────────────────────╯" );
	}
	
private:
	log_stack_t entries;
	u16         x=0, y=0;
};

struct keyboard_widget_t final: public widget_i
{
	keyboard_widget_t () noexcept = default;
	
	keyboard_widget_t ( u8 const *port, u16 x, u16 y ) noexcept:
		widget_i(), port(port), x(x), y(y)
	{}
	
	~keyboard_widget_t () noexcept final = default;
	
	void
	redraw() noexcept final
	{
	}
	
	void
	update() noexcept final
	{
		assert( port );
		std::printf( "\033[?25l\033[%u;%uH" LABEL " Last keypress: " BRT_CLR "'%c'\033[0m", y, x, *port );
	}
private:
	u8 const *port = nullptr;
	u16       x=0, y=0;
};

int
main( int const argc, char const *const argv[] )
{
	bool  is_running    = true;
	bool  is_stepping   = true;
	bool  should_step   = false;
	auto  system        = system_t                  {};
	u8   *keyboard_port = system.get_ram()+0xFF01;
	u8   *display_block = system.get_ram()+0x1000;
	auto  logger        = log_widget_t<8>           {                      87,  2 };
	logger.push( "Initializing..." );
	logger.push( "Loading widgets... ");
	auto  terminal      = terminal_display_t<80,25> { display_block,        3,  2 };
	auto  dbg_cpu       = cpu_debug_82x4_display_t  { &system.get_cpu(),    2, 28 };
	auto  history       = history_log_t<18>         { &system,              2, 32 };
	auto  stk_display   = stack_hex_display_t       { &system,             87, 32 };
	auto  zpg_display   = ram_page_hex_display_t    { &system, 0x00,       87, 12 };
	auto  prg_display   = program_hex_display_t     { &system,             32, 32 };
	auto  keyboard_w    = keyboard_widget_t         { keyboard_port,        0,  0 };
	auto  keyboard      = keyboard_t                { keyboard_port, &is_running, &is_stepping, &should_step };
	
	logger.push( "Widgets loaded!" );
	
	system.get_cpu().Hz = argc > 3? std::atoi(argv[3]) : 500;
	
	widget_i *widgets[] {
		&terminal,
		&dbg_cpu,
		&history,
		&stk_display,
		&zpg_display,
		&prg_display,
		&logger,
		&keyboard_w
	};
	
	auto txt = " xx Hello world!"; // TODO: remove
	std::memcpy( (void*)display_block, txt, 16 );
	
	logger.push( "Loading program machine code..." );
	u16 prg_start_addr = argc > 2? std::atoi(argv[2]) : 0x0300; // TODO: refactor into optional PPM define
	
	if ( argc > 1 ) {
		std::printf( "Trying to open '%s' @ 0x%04" PRIX16 "...\n", argv[1], prg_start_addr );
		auto *f = std::fopen( argv[1], "rb" );
		if ( not f ) {
			std::fprintf( stderr, "[ERROR]: Unable to open file \"%s\"!", argv[1] );
			return -1;
		}
		
		std::fseek( f, 0L, SEEK_END );
		std::size_t size = std::ftell(f) - 1; // TODO: verify ...
		std::fseek( f, 0L, SEEK_SET );
		if ( size + prg_start_addr > 0xFFFF ) {
			std::fprintf( stderr, "[ERROR]: Reading binary data at \"%s\" into RAM starting at address"
			              "$%04" PRIX16 "would exceed RAM capacity!", argv[1], prg_start_addr );
			return -2;
		}
		std::fread( system.get_ram()+prg_start_addr, size, 1, f );
		char msg[256];
		std::sprintf( msg, "Loaded '%s' into $%04" PRIX16, argv[1], prg_start_addr );
		logger.push( msg );
		system.get_cpu().PC = *(u16*)(system.get_ram() + 0xFFFC);
	}
	else {
		/*
		u8 prg[] = {
			0xEA, 0x4A, 0x09, 0x31, 0x0D, 0x20, 0x04, 0x1D,
			0x32, 0x32, 0x19, 0x90, 0x16, 0x05, 0x12, 0x15,
			0x69, 0x96, 0x42, 0x01, 0x33, 0x11, 0x24, 0x4C,
			0x00, 0x03
		};
		*/
		
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
		std::memcpy( (void*)(system.get_ram()+prg_start_addr), prg, sizeof(prg) );
	}
	auto  app           = tui_app_t                 {};
	
	logger.push( "Starting IO thread" );
	auto io_thread = std::jthread(
		[&is_running, &keyboard] {
			while ( is_running ) {
				keyboard.update();
				std::this_thread::sleep_for( 5ms );
			}
		}
	);
	
	logger.push( "Starting 6502 simulation thread" );
	auto cpu_thread = std::jthread(
		[&is_running,&is_stepping,&should_step,&system] {
			while ( is_running )
				if ( not is_stepping )
					system.update();
				else if ( should_step ) {
					system.update();
					should_step = false;
				}
		}
	);
	
	logger.push( "Starting TUI thread" );
	auto const target_framerate = (argc>4? std::atoi(argv[4]) : 30);
	auto const frame_length_ms  = 1000ms / target_framerate;
	auto gui_thread = std::jthread(
		[&is_running, &widgets, &frame_length_ms] {
			while ( is_running ) {
				curses::refresh();
				if ( false ) // TODO
					for ( auto *w: widgets )
						w->redraw();
				for ( auto *w: widgets )
					w->update();
				std::this_thread::sleep_for(frame_length_ms);
			}
		}
	);

	logger.push( "Initialization finished!" );
	
	while ( is_running ) {
		std::this_thread::sleep_for( 5ms );
	}
	
	logger.push( "Exit signal received. Exiting..." );
	// TODO: save logs to file
	std::this_thread::sleep_for(1s);	// temp
	return 0;
}

