#include <cstdint>
#include <cinttypes>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>

#include <optional>
#include <string>
#include <chrono>
#include <vector>
#include <thread>
#include <utility>
#include <memory>
#include <unordered_map>

namespace curses {
	#include "curses.h" // TODO: statically link?
}

struct tui_app_t final
{
	tui_app_t()
	{
		curses::initscr();
		curses::cbreak();
		curses::noecho();
		curses::keypad( curses::stdscr, TRUE );
		// keyboard IO thread
		// widgets TUI thread
	}
	
	~tui_app_t() noexcept
	{
		curses::endwin();
	}
private:
};

using namespace::std::literals;

// TODO: suffix with CLR?
#define LABEL                  "\033[1;33m"
#define INACTIVE_OUTER_BORDER  "\033[38;5;231m"
#define INACTIVE_INNER_BORDER  "\033[38;5;231m"
#define ACTIVE_OUTER_BORDER    "\033[38;5;222m"
#define ACTIVE_INNER_BORDER    "\033[38;5;222m"
#define CROSS_CLR              "\033[48;5;237m"
#define DIM_CLR                "\033[90m"
#define MED_CLR                "\033[38;5;248m"
#define BRT_CLR                "\033[97m"

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using i8  = std::int8_t;
using i16 = std::int16_t;

using f32 = float;

// TODO: args
//       --fps N                (   N: 1~255)
//       --hz  ADDR             (ADDR: 1~65535)
//       --bin FILEPATH ADDR    (BOOL: 0|n|N for false; 1|y|Y for true)
//       --stepmode BOOL
//       --startaddr ADDR

// TODO: add "begin writing program here" to MEM (open code editor?)
// TODO: memory wrap-around smoothly when going down?
// TODO: Memory display modes? HEX/ASCII
// TODO: Keyboard input
// TODO: Only update changes in widgets instead of reprinting them
// TODO: search memory in memory editor
// TODO: watch list for memory in memory editor (change? view?)
// TODO: breakpoints for memory in memory editor
// TODO: Colour code memory values based on access frequency and temporal proximity
// TODO: Memcopy command-line arg ROM file into system RAM
// TODO: Do more testing	
// TODO: Handle under/overflow in stack?
// TODO: Keep track of last written + clear later
// TODO: Nav mode:
//           if nav_mode:
//               if left: (A,H,←,4)
//                   prev_widget = curr_widget
//                   switch ( curr_widget ):
//                       case    term:  curr_widget = logger
//                       case hexedit:  curr_widget = term
//                       case  logger:  curr_widget = hexedit
//           
//               if right: (D,L,→,6)
//                   prev_widget = curr_widget
//                   switch ( curr_widget ):
//                       case    term:  curr_widget = hexedit
//                       case hexedit:  curr_widget = logger
//                       case  logger:  curr_widget = term
//                       
//               if up|down: (S,J,↓,2, W,K,↑,8)
//                   if curr_widget == history:
//                       curr_widget = prev_widget
//                   else:
//                       prev_widget = curr_widget
//                       curr_widget = history
//
// TODO: Memory tracking mode? Keeps the memory editor centered on the latest memory write
// TODO: Page transitions? Dashed lines + page number in the center
// TODO: Centered dots instead of '?' in hex editor?
// TODO: Breakpoints? Integrate with the memory editor

/* DEPRECATED
[[nodiscard]] inline auto
timer() noexcept -> f32
{
	using clock_t = std::chrono::high_resolution_clock;
	static clock_t::time_point            start   = clock_t::now();
	std::chrono::duration<f32,std::milli> elapsed = clock_t::now() - start;
	return elapsed.count();
};
*/

// TODO: refactor
namespace key {
	[[nodiscard]] auto constexpr
	is_up( int key ) noexcept -> bool
	{
		if ( key == 'w' or key == 'W' ) return true; // WASD
		//if ( key == 'k' or key == 'K' ) return true; // Vi hjkl
		if ( key == KEY_UP            ) return true; // arrow keys
		return false;
	}
	
	[[nodiscard]] auto constexpr
	is_left( int key ) noexcept -> bool
	{
		if ( key == 'a' or key == 'A' ) return true; // WASD
		//if ( key == 'h' or key == 'H' ) return true; // Vi hjkl
		if ( key == KEY_LEFT          ) return true; // arrow keys
		return false;
	}
	
	[[nodiscard]] auto constexpr
	is_right( int key ) noexcept -> bool
	{
		if ( key == 'd' or key == 'D' ) return true; // WASD
		//if ( key == 'l' or key == 'L' ) return true; // Vi hjkl
		if ( key == KEY_RIGHT         ) return true; // arrow keys
		return false;
	}
	
	[[nodiscard]] auto constexpr
	is_down( int key ) noexcept -> bool
	{
		if ( key == 's' or key == 'S' ) return true; // WASD
		//if ( key == 'j' or key == 'J' ) return true; // Vi hjkl
		if ( key == KEY_DOWN          ) return true; // arrow keys
		return false;
	}
} // end-of-namespace key

struct keyboard_t {
	// TODO: implement!
	// TODO: handle meta keys
	// TODO: handle up/down/pressed?
	auto
	get_input() noexcept -> std::optional<int>
	{
		assert( port );
		int key_pressed = wgetch( curses::stdscr );
		*port = key_pressed; // TODO: handle multi-byte?
		if ( key_pressed )
			return key_pressed;
		else return {};
	}
	u8   *port            = nullptr;
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
			// TEMP FIX:
			for ( u64 i=0; i<_capacity; ++i)
				_data[i] = other._data[i];
			// TODO: fix old version:
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
#define SYM_CLR "\033[1;90m"
#define IMM_CLR "\033[1;31m"
#define ZPG_CLR "\033[1;37m"
#define HEX_CLR "\033[1;93m"
#define NUM_CLR "\033[1;97m"
#define REG_CLR "\033[1;36m"
		//std::fprintf( stderr, "at(op_code: %02" PRIX8 ")\n", op_code );
		try {
			auto const info = op_meta_tbl.at(op_code); // TODO: handle exception?
			std::printf( "\033[1;33m%.3s ", info.mnemonic ); // print instruction
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
			std::printf("\033[1;31mERR INVALID " );
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



enum struct widget_type_e: u8 {
	terminal,
	logger,
	keyboard,
	instruction_history,
	memory_editor,
	addr_prompt,
	num_prompt,
	program_execution_viewer,
	stack_viewer,
	ram_page_viewer,
	cpu_info,
	newbie_info
};

struct widget_i
{
	virtual ~widget_i() {}
	virtual auto process_input( int ) -> bool = 0;
	virtual void redraw( bool is_active )     = 0;
	virtual void draw(   bool is_active )     = 0;
	virtual void update( bool is_active )     = 0;
	[[nodiscard]] virtual auto get_widget_type() const noexcept -> widget_type_e = 0;
};



struct vertical_scrollbar_widget_t // final: public widget_i
{
/*
	void
	update( bool is_active ) noexcept final
	{
	}
	
	auto
	process_input( int key ) -> bool final
	{
		return false;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
*/
	void
	draw( /*bool is_active*/ ) noexcept // final
	{
		if ( curr_top_row == 0 and height >= total_rows ) // TODO: verify
			return;
		else {
			auto const  slider_height = (u16)(((f32)height/total_rows)*height);
			auto const  remainder     = height - slider_height;
			auto const  max_top_row   = total_rows - height;
			auto const  gap_above     = (u16)((f32)curr_top_row / max_top_row * remainder);
			auto const  gap_below     = height - slider_height - gap_above - 1;
//			std::printf( "\033[0m\033[0;30H H:%u, TR:%u, SH:%u, R:%u, MTR:%u, GA:%u, GB:%u", height, total_rows, slider_height, remainder, max_top_row, gap_above, gap_below); // DEBUG

			std::printf( "\033[0m\033[%u;%uH", y, x ); // go to top
			auto curr_row = 0;
			if ( gap_above > 0 )
				std::printf( "%s%s\033[%u;%uH", rail_color, rail_top, y+(++curr_row), x );
			for ( u16 row=1; row<gap_above; ++row )
				std::printf( "%s%s\033[%u;%uH", rail_color, rail_mid, y+(++curr_row), x );
			
			std::printf( "%s%s\033[%u;%uH", slider_color, slider_top, y+(++curr_row), x );
			for ( u16 row=1; row<=slider_height; ++row )
				if ( row<slider_height )
					std::printf( "%s%s\033[%u;%uH", slider_color, slider_mid, y+(++curr_row), x );
				else
					std::printf( "%s%s\033[%u;%uH", slider_color, slider_bot, y+(++curr_row), x );
			
			for ( u16 row=1; row<=gap_below; ++row ) 
				if ( row != gap_below )
					std::printf( "%s%s\033[%u;%uH", rail_color, rail_mid, y+(++curr_row), x );
				else
					std::printf( "%s%s", rail_color, rail_bot );
			
			assert( curr_row <= height );
		}
	}
	
	u16 x, y, height, curr_top_row, total_rows;
	char const *rail_top     = "│"; //"┃"; //"╻";
	char const *rail_mid     = "│"; //"┃";
	char const *rail_bot     = "│"; //"┃"; //"╹";
	char const *rail_color   = ACTIVE_OUTER_BORDER;
	char const *slider_color = BRT_CLR; 
	char const *slider_top   = "█";
	char const *slider_mid   = "█";
	char const *slider_bot   = "█";
};



/* DEPRECATED
struct cpu_debug_15x10_display_t final: public widget_i
{
	cpu::cpu_t *CPU = nullptr;
	u8 x=0, y=0;
	
	virtual void
	draw() noexcept
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

struct cpu_info_widget_t final: public widget_i
{
	cpu_info_widget_t() noexcept = default;
	
	cpu_info_widget_t( cpu::cpu_t const *CPU, u8 x, u8 y ) noexcept:
		widget_i(), CPU(CPU), x(x), y(y)
	{}
	
	~cpu_info_widget_t() noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::cpu_info;
	}
	
	void
	update( bool is_active ) noexcept final
	{
	}
	
	auto
	process_input( int input ) -> bool final
	{
		return false;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
	
	void
	draw( bool is_active ) noexcept final
	{
		using namespace cpu;
		
		auto constexpr set    = "\033[1;32m1\033[0m";
		auto constexpr unset  = "\033[1;31m0\033[0m";
		
		assert( CPU );
		
		auto cc_digits = 1;
		auto tmp = CPU->cc;
		while ( tmp /= 10 )
			++cc_digits;
		
		std::printf( "\033[?25l\033[%u;%uH" "%s"
			"╭────┰────────────────┰──────┰────┰────┰────┰────┰──────────┰──────────────────────╮"  "\033[B\033[84D"
			"│ " LABEL    "F:"       "%s ┃ " LABEL "N V BB D I Z C"   "%s ┃ " LABEL   "PC: "       "%s ┃ " LABEL    "S:"       "%s ┃ " LABEL   "A:"        "%s ┃ " LABEL   "X:"        "%s ┃ " LABEL   "Y:"        "%s ┃ " LABEL   "Clk.Spd:"                "%s ┃ " LABEL   "Elapsed Cycle Count:"    "%s │" "\033[B\033[84D"
			"│ " BRT_CLR "%02" PRIX8 "%s ┃ " "%s %s %s%s %s %s %s %s" "%s ┃ " BRT_CLR "%04" PRIX16 "%s ┃ " BRT_CLR "%02" PRIX8 "%s ┃ " BRT_CLR "%02" PRIX8 "%s ┃ " BRT_CLR "%02" PRIX8 "%s ┃ " BRT_CLR "%02" PRIX8 "%s ┃ " BRT_CLR "%5" PRIu16 MED_CLR " Hz" "%s ┃ " DIM_CLR "%0*d" BRT_CLR "%" PRIu64 "%s │" "\033[B\033[84D"
			"╰────┸────────────────┸──────┸────┸────┸────┸────┸──────────┸──────────────────────╯",
			y, x,
			(is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER),
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			(is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER),
			CPU->P,
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			CPU->read_flag( N)?set:unset, CPU->read_flag( V)?set:unset,
			CPU->read_flag(BU)?set:unset, CPU->read_flag(BL)?set:unset,
			CPU->read_flag( D)?set:unset, CPU->read_flag( I)?set:unset,
			CPU->read_flag( Z)?set:unset, CPU->read_flag( C)?set:unset,
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			CPU->PC,
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			CPU->SP,
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			CPU->A,
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			CPU->X,
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			CPU->Y,
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			CPU->Hz,
			(is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER),
			20-cc_digits, 0, CPU->cc,
			(is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER)
		);
	}
private:
	cpu::cpu_t const *CPU = nullptr;
	u8 x=0, y=0;
};



// removes most unwanted ASCII-symbols (returns 0xFF character if encountered)
[[nodiscard]] inline auto
is_presentable_symbol( u8 const c ) noexcept -> bool
{
	return c > 0x1F and c < 0x7F;
}



constexpr char unpresentable_symbol_terminal[]   = "?";
constexpr char unpresentable_symbol_hex_editor[] = "·";

template <u8 width, u8 height>
struct terminal_widget_t final: public widget_i
{
	terminal_widget_t() noexcept = default;
	
	terminal_widget_t( u8 const *block_start, u8 x, u8 y ):
		block_start(block_start), x(x), y(y)
	{}
	
	~terminal_widget_t() noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::terminal;
	}
	
	void
	update( bool is_active ) noexcept final
	{
	}
	
	auto
	process_input( int input ) -> bool final
	{
		return false;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
	
	void
	draw( bool is_active ) noexcept
	{
		static char hdr[3*(width+2)+1],
		            ftr[3*(width+2)+1];
		
		static bool all_initialized = {
			init_hdr(hdr,width) and
			init_ftr(ftr,width)
		}; (void) all_initialized;
		
		constexpr u16 last_row_addr = (height) * width;
		
		assert( block_start );
		std::printf( "\033[?25l\033[%u;%uH" "%s" "%s" "\033[B\033[%uD", y, x, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), hdr, width+2 ); 
		for ( u16 row_addr_offset  = 0;
		          row_addr_offset  < last_row_addr;
		          row_addr_offset += width )
		{
			std::printf( "│" "\033[0m" );
			bool is_end_met = false;
			for ( u16 col=0; col<width; ++col ) {
				u8 c = block_start[row_addr_offset+col]; // raw byte
				if ( c == '\n' or c == '\0' ) 
					is_end_met = true;
				if ( not is_presentable_symbol(c) )
					std::printf( "\033[1;31m" "%s" "\033[0m", unpresentable_symbol_terminal );
				else
					std::printf( "%c", is_end_met? ' ' : c );
			}
			std::printf( "%s" "│" "\033[B\033[%uD", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), width+2 );
		}
		std::printf( "%s" "%s", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), ftr );
	}
private:
	u8 const *block_start = nullptr;
	u8        x=0, y=0;
};



struct instruction_history_widget_t final: public widget_i
{
	instruction_history_widget_t() noexcept = default;
	
	instruction_history_widget_t( system_t const *system, u16 rows, u8 x, u8 y ):
		widget_i(), system(system), rows(rows), x(x), y(y)
	{}
	
	~instruction_history_widget_t() noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::instruction_history;
	}
	
	void
	update( bool is_active ) noexcept final
	{
		scrollbar.y      = y+1;
		scrollbar.x      = x+width+1;
		scrollbar.height = rows;
	}
	
	auto
	process_input( int key ) -> bool final
	{
		assert( system );
		switch( key ) {
			case KEY_UP    : { _dec_row(1);     break; } 
			case KEY_DOWN  : { _inc_row(1);     break; }
			case KEY_PPAGE : { _dec_row(rows);  break; }
			case KEY_NPAGE : { _inc_row(rows);  break; }
			case KEY_HOME  : { curr_row=0;      break; }
			case KEY_END   : {
				curr_row=system->get_history().size();
				if ( curr_row )
					--curr_row;
				break;
			}
		}
		return false;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
	
	void
	draw( bool is_active ) noexcept
	{
		assert( system );
		
		update(is_active); // TEMP
	/*	
		std::printf( "\033[?25l\033[%u;%uH" "%s" "╭───────────────────────────╮" "\033[B\033[%uG", y, x, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), x ); 
		
		u8 entries_to_print = system->get_history().size();
		if ( entries_to_print > rows )
			entries_to_print = rows;
		
		for ( u8 i=0; i<entries_to_print; ++i ) {
			auto const e = system->get_history().peek(i);
			std::printf( "%s" "│" "%s" "\033[38m" "%04" PRIX16 " ", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), (i==0? CROSS_CLR : "\033[0m"), e.addr );
			system_t::print_asm( e.op, e.arg1, e.arg2 );
			std::printf( "\033[38;5;240m" " ;" "\033[90m%" PRIu8 " cycles" "\033[0m" "%s" "│" "\033[B\033[%uG", e.cycles, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), x );
		}
		for ( u8 i=rows-entries_to_print; i; --i )
			std::printf( "\033[0m" "%s" "│                           │" "\033[B\033[%uG", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), x );
		std::printf(    "%s" "╰───────────────────────────╯", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
	
*/
		// clearing & border
		auto const *outer_border_color = (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER );
		std::printf( "\033[%u;%uH" "%s" "╭───────────────────────────╮" "\033[B\033[%uG", y, x, outer_border_color, x ); 
		for ( u16 row=0; row<rows; ++row )
			std::printf( "%s│%*s%s│\033[B\033[%uG", outer_border_color, width, "", outer_border_color, x );
		std::printf( "%s╰───────────────────────────╯", outer_border_color );
	
		auto const &history = system->get_history();
		u8 entries_to_print = history.size() - curr_row;
		if ( entries_to_print > rows )
			entries_to_print = rows;
			
		// printing actual messages
		std::printf( "\033[%u;%uH", y+1, x+1 ); 
		for ( u8 i=0; i<entries_to_print; ++i ) {
			auto const &e = history.peek(curr_row+i);
			std::printf( "%s" "\033[38m" "%04" PRIX16 " ", (i==0? CROSS_CLR : "\033[0m"), e.addr );
			system_t::print_asm( e.op, e.arg1, e.arg2 );
			std::printf( "\033[38;5;240m" " ;" "\033[90m%" PRIu8 " cycles" "\033[0m" "\033[B\033[%uG", e.cycles, x+1 );
			/*
			std::printf( "%s" "│" "%s" "\033[1;33m%s" "%*s\033[%uG" "\033[37m%-.*s", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), (i==0? CROSS_CLR : ""), e.timestamp(), width-9, "", x+11, width-9, e.msg() );
			std::printf( "\033[%uG" "\033[0m" "%s" "│" "\033[B\033[%uG", x+width+1, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), x );*/
		}
		
		scrollbar.curr_top_row = curr_row;
		scrollbar.total_rows   = history.size() + rows;
		if ( entries_to_print and is_active )
			scrollbar.draw();
	}
	
	inline void
	set_rows( u16 const new_rows ) noexcept
	{
		rows = new_rows;
	}
	
	[[nodiscard]] inline auto
	get_rows() const noexcept -> u16
	{
		return rows;
	}
	
private:
	static u16 constexpr width = 27;
	
	vertical_scrollbar_widget_t  scrollbar;
	system_t const              *system   = nullptr;
	u16                          rows     = 0;
	u8                           x        = 0,
	                             y        = 0;
	u16                          curr_row = 0;
	
	inline void
	_dec_row( u16 const n ) noexcept
	{
		if ( n < curr_row )
			curr_row -= n;
		else
			curr_row = 0;
	}
	
	inline void
	_inc_row( u16 const n ) noexcept
	{
		assert( system );
		auto const &history = system->get_history();
		curr_row += n;
		if ( history.size() == 0 )
			curr_row = 0;
		else if ( curr_row >= history.size() )
			curr_row = history.size() - 1;
	}
};



struct stack_viewer_widget_t final: public widget_i
{
	stack_viewer_widget_t() noexcept = default;
	
	stack_viewer_widget_t( system_t const *system, u16 x, u16 y ) noexcept:
		widget_i(), system(system), x(x), y(y)
	{}
	
	~stack_viewer_widget_t() noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::stack_viewer;
	}
	
	void
	update( bool is_active ) noexcept final
	{
	}
	
	auto
	process_input( int input ) -> bool final
	{
		return false;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
	
	void
	draw( bool is_active ) noexcept final
	{
#		define GOTO_NEXT_LINE   "\033[B\033[54D" 
		const u16 page_base_addr = 0x0100;
		assert( system );
		auto const &CPU = system->get_cpu();
		auto const *RAM = system->get_ram();
		u16  const stack_end = 0x0100 + CPU.SP;
		
		std::printf( "\033[?25l\033[%u;%uH" "%s" "╭────┰───────────────────────────────────────────────╮"
		             GOTO_NEXT_LINE "│" LABEL " STK", y, x, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
		
		std::printf( "%s" "┃", (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
		for ( u8 col=0; col<0x10; ++col )
			std::printf( DIM_CLR "0"  "%s%" PRIX8 "%s", (page_base_addr+col == (CPU.PC&0xFF0F)? LABEL : BRT_CLR), col, (col==0xF? "":" ") );
		std::printf( "%s" "│" GOTO_NEXT_LINE "┝" "%s" "━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" "%s" "┥" GOTO_NEXT_LINE, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER), (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
		u16 curr_addr = page_base_addr;
		for ( u8 row=0; row<0x10; ++row ) {
			std::printf( "%s" "│" "\033[90m" "01" BRT_CLR "%1" PRIX8 DIM_CLR "0" "%s" "┃", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), row, (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
			for ( u8 col=0; col<0x10; ++col, ++curr_addr ) {
				auto const curr_byte = RAM[curr_addr];
				std::printf( "\033[%sm", curr_addr < stack_end? "1;97":"90" );
				std::printf( "%02" PRIX8 "%s", curr_byte, col==0xF?"":" " );
			}
			std::printf( "%s" "│" GOTO_NEXT_LINE, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
		}
		std::printf("╰────┸───────────────────────────────────────────────╯");
#		undef GOTO_NEXT_LINE
	}
private:
	system_t const *system = nullptr;
	u16 x=0, y=0;
};



struct ram_page_viewer_widget_t final: public widget_i
{
	ram_page_viewer_widget_t() noexcept = default;
	
	ram_page_viewer_widget_t( system_t const *system, u8 page_no, u16 x, u16 y ) noexcept:
		widget_i(), system(system), page_no(page_no), x(x), y(y)
	{}
	
	~ram_page_viewer_widget_t() noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::ram_page_viewer;
	}
	
	void
	update( bool is_active ) noexcept final
	{
	}
	
	auto
	process_input( int input ) -> bool final
	{
		return false;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
	
	void
	draw( bool is_active ) noexcept final
	{
#		define GOTO_NEXT_LINE   "\033[B\033[54D" 
		const u16 page_base_addr = page_no * 0x100;
		assert( system );
		
		std::printf( "\033[?25l\033[%u;%uH" "%s" "╭────┰───────────────────────────────────────────────╮"
		             GOTO_NEXT_LINE "│" LABEL, y, x, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
		if      ( page_no == 0 )
			std::printf( " ZPG" );
		else if ( page_no == 1 )
			std::printf( " STK" );
		else
			std::printf( "pg%02" PRIX8, page_no );
		
		auto const *RAM = system->get_ram();
		
		std::printf( "%s" "┃", (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
		for ( u8 col=0; col<0x10; ++col )
			std::printf( DIM_CLR "0" BRT_CLR "%" PRIX8 "%s", col, (col==0xF? "":" ") );
		std::printf( "%s" "│" GOTO_NEXT_LINE "┝" "%s" "━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" "%s" "┥" GOTO_NEXT_LINE, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER), (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
		u16 curr_addr = page_base_addr;
		for ( u8 row=0; row<0x10; ++row ) {
			std::printf( "%s" "│" "\033[90m%02" PRIX8 "%s" "%1" PRIX8 DIM_CLR "0" "%s" "┃", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER),
			             page_no, BRT_CLR, row, (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
			for ( u8 col=0; col<0x10; ++col, ++curr_addr ) {
				auto const curr_byte = RAM[curr_addr];
				std::printf( "\033[1;%sm", curr_byte? "1;97":"90" );
				std::printf( "%02" PRIX8 "%s", curr_byte, col==0xF?"":" " );
			}
			std::printf( "%s" "│" GOTO_NEXT_LINE, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
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

struct program_execution_viewer_widget_t final: public widget_i
{
	program_execution_viewer_widget_t () noexcept = default;
	
	program_execution_viewer_widget_t ( system_t const *system, u16 x, u16 y ) noexcept:
		widget_i(), system(system), x(x), y(y)
	{}
	
	~program_execution_viewer_widget_t () noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::program_execution_viewer;
	}
	
	void
	update( bool is_active ) noexcept final
	{
	}
	
	auto
	process_input( int input ) -> bool final
	{
		return false;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
	
	void
	draw( bool is_active ) noexcept final
	{
#		define GOTO_NEXT_LINE   "\033[B\033[54D" 
		assert( system );
		auto const &CPU     = system->get_cpu();
		auto const *RAM     = system->get_ram();
		auto const  PC      = CPU.last_addr;
		auto const  OP      = RAM[PC];
		u16  const  page_no = PC >> 8;
		
		std::printf( "\033[?25l\033[%u;%uH" "%s" "╭────┰───────────────────────────────────────────────╮"
		             GOTO_NEXT_LINE "│" LABEL "p." "%02" PRIX8, y, x, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), page_no );
		
		std::printf( "%s" "┃", (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
		for ( u8 col=0; col<=0xF; ++col ) {
			auto const is_active_col = col == (PC&0xF);
			std::printf( "%s" "0"  "%s%" PRIX8 "\033[0m%s", (is_active_col? CROSS_CLR MED_CLR : DIM_CLR), (is_active_col? LABEL : BRT_CLR), col, (col==0xF? "":" ") );
		}
		
		std::printf( "%s" "│" GOTO_NEXT_LINE "┝" "%s" "━━━━╋", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
		
		for ( u8 col=0; col<=0xF; ++col ) {
			if ( col == (PC&0xF) )
				std::printf( CROSS_CLR "━━" "\033[0m" "%s", (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
			else
				std::printf( "━━" );
			if ( col < 0xF )
				std::printf( "━" );
		}	
		std::printf( "┥" GOTO_NEXT_LINE );
		
		auto const op_start = PC;
		auto const op_end   = PC + get_op_byte_size(OP) - 1;
			
		u16 curr_addr = PC & 0xFF00;
		for ( u8 row=0; row<=0xF; ++row ) {
			bool is_active_row = (curr_addr&0xFFF0)==(PC&0xFFF0);
			std::printf( "%s" "│" "%s" "%02" PRIX8 "%s%1" PRIX8 "%s" "0" "%s" "┃", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), (is_active_row? CROSS_CLR "\033[1;33m" : "\033[90m"), page_no, (is_active_row? LABEL : BRT_CLR), row, (is_active_row? MED_CLR : DIM_CLR), (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
			for ( u8 col=0; col<=0xF; ++col, ++curr_addr ) {
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
					std::printf( "%02" PRIX8 CROSS_CLR "%s", curr_byte, col==0xF?"\033[0m":" " );
				}
			}
			std::printf( "%s" "│" GOTO_NEXT_LINE, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
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

struct logger_widget_t final: public widget_i
{
	logger_widget_t () noexcept = default;
	
	logger_widget_t ( u16 rows, u16 capacity,  u16 x, u16 y ) noexcept:
		widget_i(), rows(rows), entries(capacity), x(x), y(y)
	{
	}
	
	~logger_widget_t () noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::logger;
	}
	
	void
	update( bool is_active ) noexcept final
	{
		scrollbar.y      = y+1;
		scrollbar.x      = x+width+1;
		scrollbar.height = rows;
	}
	
	auto
	process_input( int key ) -> bool final
	{
		switch( key ) {
			case KEY_UP    : { _dec_row(1);               break; } 
			case KEY_DOWN  : { _inc_row(1);               break; }
			case KEY_PPAGE : { _dec_row(rows);            break; }
			case KEY_NPAGE : { _inc_row(rows);            break; }
			case KEY_HOME  : { curr_row=0;                break; }
			case KEY_END   : { curr_row=entries.size()-1; break; }
		}
		return false;
	}
	
	inline void
	push( char const *msg ) noexcept
	{
		entries.push(msg);
		curr_row = 0;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
	
	void
	draw( bool is_active ) noexcept final
	{
		update(is_active); // TEMP
		// TODO: add support for line break if message is too long for the width
		static char hdr[3*(width+2)+1],
		            ftr[3*(width+2)+1];
		
		static bool all_initialized = {
			init_hdr(hdr,width) and
			init_ftr(ftr,width)
		}; (void) all_initialized;
		
		char const *border_color = is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER;
		
		// clearing
		std::printf( "\033[%u;%uH" "%s" "%s" "\033[B\033[%uG", y, x, border_color, hdr, x ); 
		for ( u16 row=0; row<rows; ++row )
			std::printf( "%s│%*s%s│\033[B\033[%uG", border_color, width, "", border_color,x );
		std::printf( "%s%s", border_color, ftr );
		
		u8 entries_to_print = entries.size() - curr_row;
		if ( entries_to_print > rows )
			entries_to_print = rows;
			
		// printing actual messages
		std::printf( "\033[%u;%uH", y+1, x ); 
		for ( u8 i=0; i<entries_to_print; ++i ) {
			auto const &e = entries.peek(curr_row+i);
			std::printf( "%s" "│" "%s" "\033[1;33m%s" "%*s\033[%uG" "\033[37m%-.*s", border_color, (i==0? CROSS_CLR : ""), e.timestamp(), width-8, "", x+11, width-9, e.msg() );
			std::printf( "\033[%uG" "\033[0m" "%s" "│" "\033[B\033[%uG", x+width+1, border_color, x );
		}
		
		scrollbar.curr_top_row = curr_row;
		scrollbar.total_rows   = entries.size() + rows;
		if ( is_active )
			scrollbar.draw();
	}
	
	inline void
	set_rows( u16 const new_rows ) noexcept
	{
		rows = new_rows;
	}
	
	[[nodiscard]] inline auto
	get_rows() const noexcept -> u16
	{
		return rows;
	}
	
private:
	static inline u16 constexpr width = 35; // 29?
	vertical_scrollbar_widget_t  scrollbar;
	u16                          rows=0,curr_row=0;
	log_stack_t                  entries;
	u16                          x=0, y=0;
	
	inline void
	_dec_row( u16 const n ) noexcept
	{
		if ( n < curr_row )
			curr_row -= n;
		else
			curr_row = 0;
	}
	
	inline void
	_inc_row( u16 const n ) noexcept
	{
		curr_row += n;
		if ( curr_row >= entries.size() )
			curr_row = entries.size() - 1;
	}
};



[[nodiscard]] constexpr auto
is_hex( u8 const c ) noexcept -> bool
{
	return (c>='0' and c<='9') or (c>='a' and c<='f') or (c>='A' and c<='F');
}



[[nodiscard]] constexpr auto
is_dec( u8 const c ) noexcept -> bool
{
	return (c>='0' and c<='9');
}



[[nodiscard]] constexpr auto
hex_to_dec( u8 const c ) -> u8
{
	if      (c>='0' and c<='9') return c - '0';
	else if (c>='a' and c<='f') return c - 'a' + 10;
	else if (c>='A' and c<='F') return c - 'A' + 10;
	else throw "Invalid input!"; // TODO: replace with proper exception
}



struct num_prompt_widget_t final: public widget_i
{
	num_prompt_widget_t( num_prompt_widget_t const &)            noexcept       = default;
	num_prompt_widget_t( num_prompt_widget_t      &&)            noexcept       = default;
	num_prompt_widget_t()                                        noexcept       = default;
	num_prompt_widget_t& operator=( num_prompt_widget_t const &) noexcept       = default;
	num_prompt_widget_t& operator=( num_prompt_widget_t      &&) noexcept       = default;
	~num_prompt_widget_t()                                       noexcept final = default;
	
	num_prompt_widget_t( u8 &num, bool is_hex_mode, u16 x, u16 y ) noexcept:
		widget_i(), is_hex_mode(is_hex_mode), num_ptr(&num), x(x), y(y)
	{
	}
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::num_prompt;
	}
	
	auto
	process_input( int key ) -> bool final
	{
		if ( _is_done )
			return true;
		
		switch (key) {
			case 'x':
			case 'X':
			case KEY_F(1): { // TODO: replace with escape
				_is_done = true;
				break;
			}
			
			case KEY_BACKSPACE:
			{
				if ( is_hex_mode )
					buffer >>= 4;
				else 
					buffer /= 10;
				if ( curr_digit )
					--curr_digit;
				break;
			}
			
			case '\n':
			case '\r':
			case KEY_ENTER:
			{
				*num_ptr = buffer;
				_is_done = true;
				break;
			}
			
			default:
			{
				if ( is_hex(key) and curr_digit < 2 and is_hex_mode ) {
					buffer <<= 4;
					buffer  += hex_to_dec(key);
					++curr_digit;
				}
				else if ( is_dec(key) and curr_digit < 3 and not is_hex_mode ) {
					if ( (key-'0' + buffer*10) < 256 ) {
						buffer *= 10;
						buffer += key - '0';
						++curr_digit;
					}
				}
			}
		}
		return true;
	}
	
	void
	update( bool is_active ) noexcept final
	{
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
		auto constexpr goto_next_line =  "\033[B\033[19D";
		std::printf( "\033[%u;%uH", y, x ); // go to upper-left corner
		auto const *outer_border_clr = (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER );
		auto const *inner_border_clr = (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER );
		std::printf(
			"%s" "╭"       "─────────────────"      "╮" "%s"
			"%s" "│" LABEL "  Insert number  " "%s" "│" "%s"
			"%s" "┝" "%s"  "━━━━━━━━━━━━━━━━━" "%s" "┥" "%s",
			outer_border_clr,                                     goto_next_line,
			outer_border_clr,                   outer_border_clr, goto_next_line,
			outer_border_clr, inner_border_clr, outer_border_clr, goto_next_line
		);
		if ( is_hex_mode )
			std::printf( "%s" "│" " " "\033[2C" DIM_CLR "        00~FF " "%s" "│" "%s", outer_border_clr, outer_border_clr, goto_next_line );
		else
			std::printf( "%s" "│" " " "\033[3C" DIM_CLR  "       0~255 " "%s" "│" "%s", outer_border_clr, outer_border_clr, goto_next_line );
		std::printf( "%s" "└"         "─────────────────"      "┘", outer_border_clr );
	}
	
	void
	draw( bool is_active ) noexcept final
	{
		constexpr auto active = "\033[0;43;30m";
		constexpr auto normal = "\033[0;40;1;37m";
		redraw( is_active ); // TODO: remove when redraw system is implemented
		std::printf( "\033[%u;%uH", y+3, x+2 ); // go to start of input
		if ( is_hex_mode ) 
			std::printf( "%s" "%2" PRIX8 "\033[1D" "%s" "%1" PRIX8 "\033[0m", normal, buffer, (curr_digit<2? active : normal), (buffer&0x000F) );
		else
			std::printf( "%s" "%3" PRIu8 "\033[1D" "%s" "%1" PRIu8 "\033[0m", normal, buffer, (curr_digit<3? active : normal), (buffer%10) );
	}
	
	[[nodiscard]] auto inline
	is_done() const noexcept -> bool
	{
		return _is_done;
	}
	
private:
	bool _is_done    = false;
	bool is_hex_mode = false;
	u8  *num_ptr     = nullptr;
	u8   buffer      = 0;
	u8   x=0, y=0, curr_digit=0;
};



struct addr_prompt_widget_t final: public widget_i
{
	addr_prompt_widget_t( addr_prompt_widget_t const &)            noexcept       = default;
	addr_prompt_widget_t( addr_prompt_widget_t      &&)            noexcept       = default;
	addr_prompt_widget_t()                                         noexcept       = default;
	addr_prompt_widget_t& operator=( addr_prompt_widget_t const &) noexcept       = default;
	addr_prompt_widget_t& operator=( addr_prompt_widget_t      &&) noexcept       = default;
	~addr_prompt_widget_t()                                        noexcept final = default;
	
	addr_prompt_widget_t( u16 &addr, u16 x, u16 y ) noexcept:
		widget_i(), addr_ptr(&addr), x(x), y(y)
	{
	}
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::addr_prompt;
	}
	
	auto
	process_input( int key ) -> bool final
	{
		if ( _is_done )
			return true;
		
		switch (key) {
			case 'x':
			case 'X':
			case KEY_F(1): { // TODO: replace with escape
				_is_done = true;
				break;
			}
			
			case KEY_BACKSPACE:
			{
				buffer >>= 4;
				if ( curr_digit )
					--curr_digit;
				break;
			}
			
			case '\n':
			case '\r':
			case KEY_ENTER:
			{
				*addr_ptr = buffer;
				_is_done = true;
				break;
			}
			
			default:
			{
				if ( curr_digit < 4 and is_hex(key) ) {
					buffer <<= 4;
					buffer  += hex_to_dec(key);
					++curr_digit;
				}
			}
		}
		return true;
	}
	
	void
	update( bool is_active ) noexcept final
	{
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
		auto constexpr goto_next_line =  "\033[B\033[19D";
		std::printf( "\033[%u;%uH", y, x ); // go to upper-left corner
		auto const *outer_border_clr = (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER );
		auto const *inner_border_clr = (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER );
		std::printf(
			"%s" "╭"         "─────────────────"      "╮" "%s"
			"%s" "│" LABEL   "  Go to address  " "%s" "│" "%s"
			"%s" "┝" "%s"    "━━━━━━━━━━━━━━━━━" "%s" "┥" "%s"
		//	"%s" "│" MED_CLR " Target address: " "%s" "│" "%s"
			"%s" "│" DIM_CLR " " "\033[4C" "  0000~FFFF " "%s" "│" "%s"
			"%s" "└"         "─────────────────"      "┘",
			outer_border_clr,                                     goto_next_line,
			outer_border_clr,                   outer_border_clr, goto_next_line,
			outer_border_clr, inner_border_clr, outer_border_clr, goto_next_line,
		//	outer_border_clr,                   outer_border_clr, goto_next_line,
			outer_border_clr,                   outer_border_clr, goto_next_line,
			outer_border_clr 
		);
	}
	
	void
	draw( bool is_active ) noexcept final
	{
		constexpr auto active = "\033[0;43;30m";
		constexpr auto normal = "\033[0;40;1;37m";
		redraw( is_active ); // TODO: remove when redraw system is implemented
		std::printf( "\033[%u;%uH", y+3, x+2 ); // go to start of input
		std::printf( "%s" "%4" PRIX16 "\033[1D" "%s" "%1" PRIX16 "\033[0m", normal, buffer, (curr_digit<4? active : normal), (buffer&0x000F) );
	}
	
	[[nodiscard]] auto inline
	is_done() const noexcept -> bool
	{
		return _is_done;
	}
	
private:
	bool _is_done = false;
	u16  buffer   = 0;
	u16 *addr_ptr = nullptr;
	u8   x=0, y=0, curr_digit=0;
};

/*
struct prompt_widget_t final: public widget_i
{
	auto
	process_input( int input ) -> bool final
	{
		if ( _content ) {
			auto const child_consumed_input = _content->process_input(key) );
			return true;
		}
		if ( key == KEY_F(1) )

		return false;
	}

	inline void
	redraw( bool is_active ) noexcept final
	{
		auto const *color      = is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER;
		auto const *next_line  = "\033[B\033[71D";
		std::printf( "\033[%u;%uH" "%s" // go to upper-left corner
			"╭─────────────────╮" "%s"
			"│                 │" "%s"
			"┝━━━━━━━━━━━━━━━━━┥" "%s"
			"│                 │" "%s"
			"│                 │" "%s"
			"└─────────────────┘", y, x, color
		);
		if ( content )
			content->redraw( is_active );
	}

	inline void
	draw( bool is_active ) noexcept final
	{
		if ( is_active and not was_active_last_frame )
			redraw();
		content->draw( is_active );
	}
	
	inline void
	set_title( char const *new_title ) noexcept
	{
		_title = new_title;
		_update_width();
	}
	
private:
	bool        _was_active_last_frame = false;
	u8          _x                     = 0,
	            _y                     = 0,
	            _width                 = 3,
	            _height                = 5;
	widget_i   *_content               = nullptr;
	char const *_title                 = "";
	
	inline void
	_update_width() noexcept
	{
		auto const     old_width = _width;
		auto const   title_width =  strlen( _title ) + 2; // space padding before and after
		auto const content_width = _content? _content->width() : 0;
		_width = std::max( title_width, content_width ) + 2; // border |
		if ( old_width != _width ) {
			redraw();
			draw();
		}
	}
};
*/
/*

│ Target address: │
│ ____  0000~FFFF │
└─────────────────┘

╭─────────────╮
│ Edit memory │
┝━━━━━━━━━━━━━┥
│ New value:  │
│ __    00~FF │
└─────────────┘

╭─────────────╮
│ Edit memory │
┝━━━━━━━━━━━━━┥
│ New value:  │
│ ___   0~255 │
└─────────────┘


│F200┃00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00┃················│
│F210┃00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00┃················│
│F220┃00 00 00 00 00 00 00 ................0 00 00 00┃················│
│F230┃00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00┃················│
│F240┃00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00┃················│
70


70-16=54
54/2 = 27
*/

template <u8 rows>
struct memory_editor_widget_t final: public widget_i
{
	memory_editor_widget_t ( memory_editor_widget_t const & ) = delete;
	memory_editor_widget_t ( memory_editor_widget_t      && ) = default;
//		widget_i(), system(system), x(x), (y) {}
	
	memory_editor_widget_t () noexcept = default;
	
	memory_editor_widget_t ( system_t *system, u16 x, u16 y ) noexcept:
		widget_i(), system(system), x(x), y(y)
	{
		_ensure_in_display_bounds();
	}
	
	~memory_editor_widget_t () noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::memory_editor;
	}
	
	void
	update( bool is_active ) noexcept final
	{
		assert( system );
		if ( addr_prompt ) {
			addr_prompt->update( is_active );
			if (addr_prompt->is_done() ) {
				addr_prompt = nullptr; // close
				_ensure_in_display_bounds();
			}
		}
		if ( num_prompt ) {
			num_prompt->update( is_active );
			if (num_prompt->is_done() ) {
				num_prompt = nullptr; // close
				_ensure_in_display_bounds();
			}
		}
	}
	
	auto
	process_input( int key ) -> bool final
	{
		assert( system );
		
		if ( addr_prompt )
			return addr_prompt->process_input( key );
		else if ( num_prompt )
			return num_prompt->process_input( key );
		else switch( key ) {
			case KEY_LEFT  : { _dec_addr(0x001); goto end; }
			case KEY_RIGHT : { _inc_addr(0x001); goto end; }
			case KEY_UP    : { _dec_addr(0x010); goto end; }
			case KEY_DOWN  : { _inc_addr(0x010); goto end; }
			case KEY_PPAGE : { _dec_addr(0x100); goto end; }
			case KEY_NPAGE : { _inc_addr(0x100); goto end; }
			case KEY_HOME  : { curr_addr=0x0000; _ensure_in_display_bounds(); goto end; }
			case KEY_END   : { curr_addr=0xFFFF; _ensure_in_display_bounds(); goto end; }
		}
		if ( is_edit_mode ) {
			if ( is_hex_mode ) {
				switch ( key )
				{
					case 'd':
					case 'D':
					{
						num_prompt = std::make_unique<num_prompt_widget_t>( system->get_ram()[curr_addr], false, x+20, y+14 );
						break;
					}
					
					case 'g':
					case 'G':
					{
						addr_prompt = std::make_unique<addr_prompt_widget_t>( curr_addr, x+20, y+14 );
						break;
					}
					
					case 'h':
					case 'H':
					{
						num_prompt = std::make_unique<num_prompt_widget_t>( system->get_ram()[curr_addr], true, x+20, y+14 );
						break;
					}
					
					case 'x':
					case 'X':
					case KEY_F(1):
					{
						is_edit_mode = false;
						break;
					}
				}
			}
			else { // dec mode
				switch ( key )
				{
					case KEY_ENTER:
					case '\r':
					case '\n':
					{
						is_edit_mode = false;
						break;
					}
					
					case KEY_BACKSPACE:
					{
						system->get_ram()[curr_addr--] = ' ';
						break;
					}
					
					case KEY_DC: // delete
					{
						system->get_ram()[curr_addr] = ' '; // TODO: move subsequent (until break) ascii forward?
						break;
					}
					
					// TODO: tabs!
					
					default: // TODO: sanitize input
						system->get_ram()[curr_addr++] = key;
				}
			}
		}
		else { // not edit mode
			if      ( key::is_left  (key) ) _dec_addr(0x001);
			else if ( key::is_right (key) ) _inc_addr(0x001);
			else if ( key::is_up    (key) ) _dec_addr(0x010);
			else if ( key::is_down  (key) ) _inc_addr(0x010);
			else switch ( key )
			{
				case 'g':
				case 'G':
				{
					addr_prompt = std::make_unique<addr_prompt_widget_t>( curr_addr, x+20, y+14 );
					break;
				}
				
				case 'm':
				case 'M':
				{
					is_hex_mode = not is_hex_mode;
					break;
				}
				
				case 'i':
				case 'I':
				case 'e':
				case 'E':
				{
					is_edit_mode = true;
					break;
				}
			}
		}
	//	else if ( 'b' == key ) brk_tbl.at(curr_addr) = not brk_tbl.at(curr_addr);
	//	TODO: add to watch list?
		end: return true; // monopolize input
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
		if ( num_prompt )
			num_prompt->redraw( is_active );
		if ( addr_prompt )
			addr_prompt->redraw( is_active );
	}

	void
	draw( bool is_active ) noexcept final
	{
		if ( num_prompt ) {
			_draw_frame( false );
			num_prompt->draw( is_active );
			return;
		}
		else if ( addr_prompt ) {
			_draw_frame( false );
			addr_prompt->draw( is_active );
			return;
		}
		// TODO:
		//       top_marginal, btm_marginal
		//       side movement... wrap? next line? block?
		//       confirm edit?
#		define ACTIVE_UNPRESENTABLE_CLR   "\033[38;5;1m"
#		define INACTIVE_UNPRESENTABLE_CLR "\033[38;5;1m"
		assert( system );
		auto const *RAM = system->get_ram();
		u8 curr_col = curr_addr & 0xF;

		_draw_frame( is_active );
	// header:
		std::printf( "\033[%u;%uH" LABEL "%s" "\033[1C", y+1, x+1, (is_edit_mode? "EDIT":"VIEW") );
		// hex cols:
		for ( u8 col=0; col<=0xF; ++col )
			std::printf( "%s" "%01" PRIX8 "\033[0m" "%s", (is_hex_mode and (col==curr_col)? CROSS_CLR MED_CLR "0" LABEL : DIM_CLR "0" BRT_CLR), col, (col==0xF? "\033[1C":" ") );
		// ascii cols:
		for ( u8 col=0; col<=0xF; ++col )
			std::printf( "%s" "%s" "%01" PRIX8 "\033[0m", (not is_hex_mode and (col==curr_col)? CROSS_CLR:""), (not is_hex_mode and (col==curr_col)? LABEL : BRT_CLR), col );
/*
		std::printf( "\033[%u;%uH", y+2, x+1 );
		// hex cols:
		for ( u8 col=0; col<=0xF; ++col )
			std::printf( "%s" "━━" "\033[0m" "%s" "%s", (is_hex_mode and (col==curr_col)? CROSS_CLR : ""), (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER), (col<0xF? "━":"╋") );
		// ascii cols:
		for ( u8 col=0; col<=0xF; ++col )
			std::printf( "%s" "━" "\033[0m" "%s", (not is_hex_mode and (col==curr_col)? CROSS_CLR : ""), (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
		std::printf( "%s" "┥"  GOTO_NEXT_LINE, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
*/
	// main rows:
		u16  curr_row_addr = top_row_addr;
		//auto prev_row_addr = curr_row_addr;
		for ( u16 row=0; row<rows; ++row ) {
			std::printf( "\033[%u;%uH", y+3+row, x );
/*
			bool const crossed_pages = (prev_row_addr&0xFF00) != (curr_row_addr&0xFF00);
		// dashed separator: (if about to cross over to a new page)
			if ( crossed_pages ) {
				std::printf( "%s" "│" "••••" "%s" "┠", (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER), (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
				// hex cols:
				for ( u8 col=0; col<=0xF; ++col )
					std::printf( "%s" "┄┄" "\033[0m" "%s" "%s", (is_hex_mode and (col==curr_col)? CROSS_CLR : ""), (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER), (col==0xF?"":"┄") );
				std::printf( "╂" );
				// ascii cols:
				for ( u8 col=0; col<=0xF; ++col ) {
					if ( (col==curr_col) and not is_hex_mode )
						std::printf( CROSS_CLR "┄" "\033[0m" "%s", (is_active? ACTIVE_INNER_BORDER : INACTIVE_INNER_BORDER) );
					else
						std::printf( "┄" );
				};
				std::printf( "%s" "┤" GOTO_NEXT_LINE, (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER) );
				prev_row_addr = curr_row_addr;
				continue; // continue printing rows
			}
*/
		// otherwise regular memory data row:
			bool const is_active_row = (curr_addr&0xFFF0) == curr_row_addr;
			auto const *bg_col = (curr_row_addr>>8) % 2? "\033[48;5;239m" : "\033[48;5;240m"; // TODO: refactor out?
			std::printf( "\033[1C" "%s" "%s" "%03" PRIX16 "%s" "\033[1C", bg_col, (is_active_row? CROSS_CLR LABEL : BRT_CLR), (curr_row_addr>>4), (is_active_row? MED_CLR "0" : DIM_CLR "0") );
			// hex cols:
			for ( u8 col=0; col<=0xF; ++col ) {
				u8  const curr_byte = RAM[curr_row_addr+col];
				if ( is_active_row )
					std::printf( "%s" "%02" PRIX8 "%s" "%s", ((col==curr_col)? (is_hex_mode? "\033[1;93;48;5;235m" : BRT_CLR) : MED_CLR), curr_byte, ((col==curr_col)? CROSS_CLR:""), col==0xF?"\033[1C":" " );
				else {
					std::printf( "%s" "%02" PRIX8 "\033[0m" "%s" "%s", (is_hex_mode? (col==curr_col? CROSS_CLR MED_CLR : MED_CLR) : DIM_CLR), curr_byte, bg_col, col==0xF?"\033[1C":" " );
				}
			}
			// ascii cols:
			for ( u8 col=0; col<=0xF; ++col ) {
				u8  const curr_byte      = RAM[curr_row_addr+col];
				u8  const is_presentable = is_presentable_symbol(curr_byte);
				if ( is_active_row ) {
					if ( col==curr_col )
						std::printf( "\033[4m" );
					
					if ( is_presentable )
						std::printf( "%s" "%c", (col==curr_col? (is_hex_mode? BRT_CLR : "\033[1;93;48;5;235m") : (is_hex_mode? MED_CLR : BRT_CLR)), curr_byte );
					else
						std::printf( ACTIVE_UNPRESENTABLE_CLR "%s" "%s", (col==curr_col? (not is_hex_mode? "\033[48;5;234m":"") : INACTIVE_UNPRESENTABLE_CLR), unpresentable_symbol_hex_editor );
					
					if ( col==curr_col )
						std::printf( "\033[0m" "%s" CROSS_CLR, bg_col );
				}
				else {
					if ( col==curr_col and not is_hex_mode )
						std::printf( CROSS_CLR );
					
					if ( is_presentable )
						std::printf( "%s" "%c", (is_hex_mode? DIM_CLR : BRT_CLR /*(col==curr_col? BRT_CLR : MED_CLR)*/), curr_byte );
					else
						std::printf( INACTIVE_UNPRESENTABLE_CLR "%s", unpresentable_symbol_hex_editor );
					
					if ( col==curr_col and not is_hex_mode )
						std::printf( "\033[0m" "%s", bg_col );
				}
			}	
			// prev_row_addr =  curr_row_addr;
			curr_row_addr += 0x10; // next row
		}
	}
	
private:
	// TODO: keyboard input listener
	system_t                             *system            = nullptr;
	u16                                   x                 = 0;
	u16                                   y                 = 0;
	u16                                   top_row_addr      = 0x1000; // TODO
	u16                                   curr_addr         = 0x1000; // TODO
	bool                                  is_hex_mode       = true;
	bool                                  is_edit_mode      = false;
	u8                                    min_edge_distance = 3;
	std::unique_ptr< num_prompt_widget_t> num_prompt        = nullptr;
	std::unique_ptr<addr_prompt_widget_t> addr_prompt       = nullptr;
	
	void inline
	_dec_addr( u16 const offset ) noexcept
	{
		curr_addr -= offset;
		_ensure_in_display_bounds();
	}
	
	void inline
	_inc_addr( u16 const offset ) noexcept
	{
		curr_addr += offset;
		_ensure_in_display_bounds();
	}
	
	void inline
	_draw_frame( bool is_active ) noexcept
	{
		auto constexpr goto_next_line   = "\033[B\033[71D";
		auto const    *outer_border_clr = (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER );
		auto const    *inner_border_clr = (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER );
		std::printf( "\033[%u;%uH" "%s", y, x, outer_border_clr );
		// print header:
		std::printf(    "╭"   "────┰───────────────────────────────────────────────┰────────────────"   "╮%s"
		                "│\033[4C%s┃\033[47C"                                     "┃\033[16C"         "%s│%s" 
						    "┝%s" "━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━" "%s┥%s",
						    goto_next_line,
						    inner_border_clr, outer_border_clr, goto_next_line,
						    inner_border_clr, outer_border_clr, goto_next_line );
		// body:
		for ( u8 row=0; row<rows; ++row )
			std::printf( "│\033[4C%s┃\033[47C"                                     "┃\033[16C"         "%s│%s",
							inner_border_clr, outer_border_clr, goto_next_line );
		// footer:
		std::printf(    "╰"   "────┸───────────────────────────────────────────────┸────────────────"   "╯" );
	}
	
	void inline
	_ensure_in_display_bounds() noexcept // TODO: transitions issue! special transition for end-of-RAM? PG.UP and PG.DN! Fix arrow reading
	{
/*
		auto transitions_between = []( u16 upper_addr, u16 lower_addr ) -> u16 {
			int i_lower = lower_addr >> 8;
			int i_upper = upper_addr >> 8;
			if ( i_lower >= i_upper )
				i_lower -= 0x100;
			return i_upper - i_lower;
		};
*
*
*/
		/*
		auto min_raw = (curr_addr&0xFFF0)-(min_edge_distance<<4);
		auto min_adj = min_raw + (transitions_between(curr_addr,min_raw));
		if ( top_row_addr < min_adj )
			top_row_addr = min_adj;*/
/*

		// ensure bottom row padding:
		if ( (std::abs( (int)(top_row_addr) - (curr_addr&0xFFF0) )>>4) >= (rows-min_edge_distance-1) ) {
		//else if ( std::abs( ((int)(top_row_addr>>4) + rows) - (curr_addr>>4)) < min_edge_distance+2 ) // TODO: fix!
			u16 x               = (((u16)rows - min_edge_distance - 2) << 4);
			u16 bottom_row_addr = (curr_addr&0xFFF0) + min_edge_distance;
			bottom_row_addr    -= transitions_between( bottom_row_addr, curr_addr );
			top_row_addr        = bottom_row_addr - 5;
		}
		// ensure top row padding:
		/else if ( std::abs((int)(curr_addr>>4) - (top_row_addr>>4)) < min_edge_distance ) {
			top_row_addr  = (curr_addr&0xFFF0) - ((u16)(min_edge_distance)<<4);
			top_row_addr += transitions_between( curr_addr, top_row_addr );
		}/
*/
		auto const pad_offs = min_edge_distance<<4;
		auto const row_offs = rows<<4;
		auto const curr_row = curr_addr&0xFFF0;
		if ( top_row_addr + pad_offs >= curr_row )
			top_row_addr = curr_row - pad_offs;
		else if ( top_row_addr + row_offs - pad_offs <= curr_row )
				top_row_addr = curr_row + pad_offs - row_offs + 0x10;
	}
};



struct newbie_info_widget_t final: public widget_i
{
	newbie_info_widget_t () noexcept = default;
	
	newbie_info_widget_t ( widget_i **current_widget, u16 rows, u16 x, u16 y ) noexcept:
		widget_i(), current_widget(current_widget), rows(rows), x(x), y(y)
	{}
	
	~newbie_info_widget_t () noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::newbie_info;
	}
	
	void
	update( bool is_active ) noexcept final
	{
	}
	
	auto
	process_input( int key ) -> bool final
	{
		return false;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
	
	void
	draw( bool is_active ) noexcept final
	{
		auto constexpr goto_next_line   = "\033[B\033[37D";
		auto const    *outer_border_clr = (is_active? ACTIVE_OUTER_BORDER : INACTIVE_OUTER_BORDER );
		std::printf(    "\033[%u;%uH%s"
		                "╭───────────────────────────────────╮%s", y, x, outer_border_clr, goto_next_line );
		for ( u16 row=0; row<rows; ++row )
			std::printf( "│\033[35C"                         "│%s", goto_next_line );
		std::printf(    "╰───────────────────────────────────╯" );
	}

private:
	widget_i **current_widget = nullptr;
	u16        rows=0, x=0, y=0;
};



[[nodiscard]] constexpr auto
key_to_string( int key ) noexcept
{
	switch ( key )
	{
		case KEY_F0:        return "F0";
		case KEY_F( 1):     return "F1";
		case KEY_F( 2):     return "F2";
		case KEY_F( 3):     return "F3";
		case KEY_F( 4):     return "F4";
		case KEY_F( 5):     return "F5";
		case KEY_F( 6):     return "F6";
		case KEY_F( 7):     return "F7";
		case KEY_F( 8):     return "F8";
		case KEY_F( 9):     return "F9";
		case KEY_F(10):     return "F10";
		case KEY_F(11):     return "F11";
		case KEY_F(12):     return "F12";
		case KEY_PPAGE:     return "Page Up";
		case KEY_NPAGE:     return "Page Down";
		case KEY_DOWN:      return "Down";
		case KEY_UP:        return "Up";
		case KEY_LEFT:      return "Left";
		case KEY_RIGHT:     return "Right";
		case KEY_BREAK:     return "Break";
		case KEY_HOME:      return "Home";
		case KEY_BACKSPACE: return "Backspace";
		case KEY_DL:        return "Delete Line";
		case KEY_IL:        return "Insert Line";
		case KEY_DC:        return "Delete";
		case KEY_IC:        return "Insert";
		case KEY_EIC:       return "Exit Insert Mode";
		case KEY_CLEAR:     return "Clear";
		case KEY_EOS:       return "End-of-Screen";
		case KEY_EOL:       return "End-of-Line";
		case KEY_SF:        return "Scroll Forward";
		case KEY_SR:        return "Scroll Backward";
		case KEY_STAB:      return "Set Tab";
		case KEY_CTAB:      return "Clear Tab";
		case KEY_CATAB:     return "Clear All Tabs";
		case '\r':
		case '\n':
		case KEY_ENTER:     return "Enter";
		case KEY_SRESET:    return "Soft Reset";
		case KEY_RESET:     return "Hard Reset";
		case KEY_PRINT:     return "Print";
/*
KEY_LL	Home down or bottom (lower left)
KEY_A1	Upper left of keypad
KEY_A3	Upper right of keypad
KEY_B2	Center of keypad
KEY_C1	Lower left of keypad
KEY_C3	Lower right of keypad
KEY_BTAB	Back tab key
KEY_BEG	Beg(inning) key
KEY_CANCEL	Cancel key
KEY_CLOSE	Close key
KEY_COMMAND	Cmd (command) key
KEY_COPY	Copy key
KEY_CREATE	Create key
KEY_MARK	Mark key
KEY_MESSAGE	Message key

KEY_MOUSE	Mouse event read
KEY_MOVE	Move key
KEY_NEXT	Next object key
KEY_OPEN	Open key
KEY_OPTIONS	Options key
KEY_PREVIOUS	Previous object key
KEY_REDO	Redo key
KEY_REFERENCE	Ref(erence) key
KEY_REFRESH	Refresh key
KEY_REPLACE	Replace key
KEY_RESIZE	Screen resized
KEY_RESTART	Restart key
KEY_RESUME	Resume key
KEY_SAVE	Save key
KEY_SBEG	Shifted beginning key
KEY_SCANCEL	Shifted cancel key
KEY_SCOMMAND	Shifted command key
KEY_SCOPY	Shifted copy key
KEY_SCREATE	Shifted create key
KEY_SDC	Shifted delete char key
KEY_SDL	Shifted delete line key
KEY_SELECT	Select key
KEY_SEND	Shifted end key
KEY_SEOL	Shifted clear line key
KEY_SEXIT	Shifted exit key
KEY_SFIND	Shifted find key
KEY_SHELP	Shifted help key
KEY_SHOME	Shifted home key
KEY_SIC	Shifted input key
KEY_SLEFT	Shifted left arrow key
KEY_SMESSAGE	Shifted message key
KEY_SMOVE	Shifted move key
KEY_SNEXT	Shifted next key
KEY_SOPTIONS	Shifted options key
KEY_SPREVIOUS	Shifted prev key
KEY_SPRINT	Shifted print key
KEY_SREDO	Shifted redo key
KEY_SREPLACE	Shifted replace key
KEY_SRIGHT	Shifted right arrow
KEY_SRESUME	Shifted resume key
KEY_SSAVE	Shifted save key
KEY_SSUSPEND	Shifted suspend key
KEY_SUNDO	Shifted undo key
KEY_SUSPEND	Suspend key
KEY_UNDO	Undo key
*/
		case KEY_END:       return "End";
		case KEY_EXIT:      return "Exit";
		case KEY_FIND:      return "Find";
		case KEY_HELP:      return "Help";
		case '\'':         return "\'";
		case '\\':         return "\\";
		case ' ':          return "Space";
		case '\"':         return "\"";
		case '!':          return "!";
		case '#':          return "#";
		case '$':          return "$";
		case '%':          return "%";
		case '&':          return "&";
		case '(':          return "(";
		case ')':          return ")";
		case '*':          return "*";
		case '+':          return "+";
		case ',':          return ",";
		case '-':          return "-";
		case '.':          return ".";
		case '/':          return "/";
		case ':':          return ":";
		case ';':          return ";";
		case '<':          return "<";
		case '=':          return "=";
		case '>':          return ">";
		case '?':          return "?";
		case '@':          return "@";
		case '[':          return "[";
		case ']':          return "]";
		case '^':          return "^";
		case '_':          return "_";
		case '`':          return "`";
		case '{':          return "{";
		case '|':          return "|";
		case '}':          return "}";
		case '~':          return "~";
		case '0':          return "0";
		case '1':          return "1";
		case '2':          return "2";
		case '3':          return "3";
		case '4':          return "4";
		case '5':          return "5";
		case '6':          return "6";
		case '7':          return "7";
		case '8':          return "8";
		case '9':          return "9";
		case 'a':          return "a";
		case 'b':          return "b";
		case 'c':          return "c";
		case 'd':          return "d";
		case 'e':          return "e";
		case 'f':          return "f";
		case 'g':          return "g";
		case 'h':          return "h";
		case 'i':          return "i";
		case 'j':          return "j";
		case 'k':          return "k";
		case 'l':          return "l";
		case 'm':          return "m";
		case 'n':          return "n";
		case 'o':          return "o";
		case 'p':          return "p";
		case 'q':          return "q";
		case 'r':          return "r";
		case 's':          return "s";
		case 't':          return "t";
		case 'u':          return "u";
		case 'v':          return "v";
		case 'w':          return "w";
		case 'x':          return "x";
		case 'y':          return "y";
		case 'z':          return "z";
		case 'A':          return "A";
		case 'B':          return "B";
		case 'C':          return "C";
		case 'D':          return "D";
		case 'E':          return "E";
		case 'F':          return "F";
		case 'G':          return "G";
		case 'H':          return "H";
		case 'I':          return "I";
		case 'J':          return "J";
		case 'K':          return "K";
		case 'L':          return "L";
		case 'M':          return "M";
		case 'N':          return "N";
		case 'O':          return "O";
		case 'P':          return "P";
		case 'Q':          return "Q";
		case 'R':          return "R";
		case 'S':          return "S";
		case 'T':          return "T";
		case 'U':          return "U";
		case 'V':          return "V";
		case 'W':          return "W";
		case 'X':          return "X";
		case 'Y':          return "Y";
		case 'Z':          return "Z";
		default:           return "Unknown";
	}
}



struct keyboard_widget_t final: public widget_i
{
	keyboard_widget_t () noexcept = default;
	
	keyboard_widget_t ( u8 const *port, u16 x, u16 y ) noexcept:
		widget_i(), port(port), x(x), y(y)
	{}
	
	~keyboard_widget_t () noexcept final = default;
	
	[[nodiscard]] auto
	get_widget_type() const noexcept -> widget_type_e final
	{
		return widget_type_e::keyboard;
	}
	
	void
	update( bool is_active ) noexcept final
	{
	}
	
	auto
	process_input( int key ) -> bool final
	{
		has_key  = true;
		last_key = key;
		return false;
	}
	
	void
	redraw( bool is_active ) noexcept final
	{
	}
	
	void
	draw( bool is_active ) noexcept final
	{
		if ( has_key )
			std::printf( "\033[0m\033[%u;%uH" LABEL " Last keypress: " DIM_CLR "'" BRT_CLR "%s" DIM_CLR "'\033[0m                ", y, x, key_to_string(last_key) );
	}
	
	inline void
	clear() noexcept
	{
		has_key = false;
	}
	
private:
	bool      has_key  = false;
	int       last_key = 0;
	u8 const *port     = nullptr;
	u16       x=0, y=0;
};



int
main( int const argc, char const *const argv[] )
{
	bool  is_running        = true;
	bool  is_stepping       = true;
	bool  is_in_nav_mode    = false;
	bool  should_step       = false;
	bool  is_newbie_mode    = true;
	auto  app               = tui_app_t                         {};
	auto  system            = system_t                          {};
	u8   *keyboard_port     = system.get_ram()+0xFF01;
	u8   *display_block     = system.get_ram()+0x1000;
	auto  keyboard          = keyboard_t                        { keyboard_port               };
	auto  logger            = logger_widget_t                   { 29, 80,             159,  2 };
	logger.push( "Initializing..." );
	logger.push( "Loading widgets... ");
	auto  terminal          = terminal_widget_t<80,25>          { display_block,        3,  2 };
	auto  dbg_cpu           = cpu_info_widget_t                 { &system.get_cpu(),    2, 29 };
	auto  history           = instruction_history_widget_t      { &system, 18,          2, 33 };
	auto  stk_display       = stack_viewer_widget_t             { &system,             87, 33 };
	auto  zpg_display       = ram_page_viewer_widget_t          { &system, 0x00,      142, 33 };
	auto  prg_display       = program_execution_viewer_widget_t { &system,             32, 33 };
	auto  hex_editor        = memory_editor_widget_t<27>        { &system,             87,  2 };
	auto  keyboard_w        = keyboard_widget_t                 { keyboard_port,        0,  0 };
	
	auto *active_widget     = (widget_i*)&terminal;
	auto *prev_widget       = active_widget;
	
	std::optional<newbie_info_widget_t> maybe_newbie_info = {};
	if ( is_newbie_mode ) {
		logger.set_rows( 9 );
		maybe_newbie_info = newbie_info_widget_t { &active_widget, 18, 159, 13 };
	}
	
	logger.push( "Widgets loaded!" );
	
	system.get_cpu().Hz = argc > 3? std::atoi(argv[3]) : 500;
	
	std::vector<widget_i*> widgets {
		&terminal,
		&dbg_cpu,
		&history,
		&stk_display,
		&zpg_display,
		&prg_display,
		&logger,
		&hex_editor,
		&keyboard_w
	};
	if ( maybe_newbie_info )
		widgets.push_back( (widget_i*)&maybe_newbie_info.value() );
	
	logger.push( "Loading program code..." );
	
	if ( argc > 1 ) {
		u16 prg_start_addr = argc > 2? std::atoi(argv[2]) : 0x0300; // TODO: refactor into optional PPM define
		
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
	/* 032E        STA $1001   */  0x8D, 0x01, 0x10,
	/* 0331        LDA *$00    */  0xA5, 0x00,
	/* 0333        AND #$0F    */  0x29, 0x0F,
	/* 0335        ADC #$30    */  0x69, 0x30,
	/* 0337        STA $1002   */  0x8D, 0x02, 0x10,
	/* 033A        RTS         */  0x60
		};
		u16 prg_start_addr = 0x0300; // TODO: refactor into optional PPM define
		
		u8 txt[2048] = " xx Hello world!                       this is a test                           "
		               "             :D                  ^                                              "
		               "                                 |__ this symbol should be invalid (red ?)      "
		               "                                                                                "
		               "                                                                                "
		               "                                                                                "
		               "                        abcdefghjklmnopqrstuvwxyz                               "
		               "                                                                                "
		               "                                                                                "
		               "                                                                                "
		               "                              :(                                                "
		               "                                                                                "
		               "                                                                                "
		               "                                                                                "
		               "                                                                                "
		               "                                                                                "
		               "         beep boop                                                              "
		               "                                                                                "
		               "                                                                                "
		               "                                                                                "
		               "                                                                                "
		               "                                                                                "
		               "                                                                                "
		               "                                                                             123"
		               "                                                                             xyz";
		txt[33] = 0x15; // intentional error
		std::memcpy( (void*)display_block, txt, sizeof(txt) );
		
		std::memcpy( (void*)(system.get_ram()+prg_start_addr), prg, sizeof(prg) );
		*(u16*)(system.get_ram() + 0xFFFC) = prg_start_addr;
	}
	
	system.get_cpu().PC = *(u16*)(system.get_ram() + 0xFFFC);
		
	logger.push( "Starting IO thread" );
	auto io_thread = std::jthread(
		[&] {
			while ( is_running ) {
				auto maybe_input = keyboard.get_input();
				if ( maybe_input ) {
					auto const key = *maybe_input;
					keyboard_w.process_input( key );
					
					if ( key == KEY_BREAK ) {
						is_in_nav_mode = true;
						logger.push( "Entering nav mode" );
					}
					
					if ( is_in_nav_mode ) {
						if ( key::is_left(key) ) {
							if ( active_widget != (widget_i*)&history )
								prev_widget = active_widget;
							if      ( active_widget == (widget_i*)&terminal   ) active_widget = (widget_i*)&logger;
							else if ( active_widget == (widget_i*)&hex_editor ) active_widget = (widget_i*)&terminal;
							else if ( active_widget == (widget_i*)&logger     ) active_widget = (widget_i*)&hex_editor;
						}
						
						else if ( key::is_right(key) ) {
							if ( active_widget != (widget_i*)&history )
								prev_widget = active_widget;
							if      ( active_widget == (widget_i*)&terminal   ) active_widget = (widget_i*)&hex_editor;
							else if ( active_widget == (widget_i*)&hex_editor ) active_widget = (widget_i*)&logger;
							else if ( active_widget == (widget_i*)&logger     ) active_widget = (widget_i*)&terminal;
						}
						
						else if ( key::is_up(key) or key::is_down(key) ) {
							if ( active_widget == (widget_i*)&history )
								active_widget = prev_widget;
							else {
								prev_widget   = active_widget;
								active_widget = (widget_i*)&history;
							}
						}
						
						else {
							is_in_nav_mode = false;
							logger.push( "Exiting nav mode" );
						}
					}
					
					else if ( key == KEY_SLEFT ) {
						if ( active_widget != (widget_i*)&history )
							prev_widget = active_widget;
						if      ( active_widget == (widget_i*)&terminal   ) active_widget = (widget_i*)&logger;
						else if ( active_widget == (widget_i*)&hex_editor ) active_widget = (widget_i*)&terminal;
						else if ( active_widget == (widget_i*)&logger     ) active_widget = (widget_i*)&hex_editor;
					}
					
					else if ( key == KEY_SRIGHT ) {
						if ( active_widget != (widget_i*)&history )
							prev_widget = active_widget;
						if      ( active_widget == (widget_i*)&terminal   ) active_widget = (widget_i*)&hex_editor;
						else if ( active_widget == (widget_i*)&hex_editor ) active_widget = (widget_i*)&logger;
						else if ( active_widget == (widget_i*)&logger     ) active_widget = (widget_i*)&terminal;
					}
					
					else if ( key == KEY_SF or key == KEY_SR ) {
						if ( active_widget == (widget_i*)&history )
							active_widget = prev_widget;
						else {
							prev_widget   = active_widget;
							active_widget = (widget_i*)&history;
						}
					}
					
					bool const input_was_consumed = active_widget->process_input( key );
					if ( not input_was_consumed ) {
					// TODO: refactor into tui_app_t
					//	if ( key_pressed == 'x' )  // terminate when "x" is pressed
					//		*is_running= false;
						if ( key == 's' )
							is_stepping = not is_stepping; // flip
						else if ( is_stepping and key == 'n' )
							should_step = true;
					}
				}
				else keyboard_w.clear();
				std::this_thread::sleep_for( 5ms );
			}
		}
	);
	
	logger.push( "Starting CPU sim thread" );
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
		[&is_running, &widgets, &active_widget, &frame_length_ms] {
			while ( is_running ) {
				curses::refresh();
				if ( false ) // TODO
					for ( auto *w: widgets )
						w->redraw( w==active_widget );
				for ( auto *w: widgets ) {
					w->update( w==active_widget );
					w->draw( w==active_widget );
				}
				std::this_thread::sleep_for(frame_length_ms);
			}
		}
	);

	logger.push( "Initialization finished!" );
	
	while ( is_running ) {
		std::this_thread::sleep_for( 5ms );
	}
	
	logger.push( "Exit signal received" );
	logger.push( "Exiting..." );
	// TODO: save logs to file
	std::this_thread::sleep_for(1s); // temp
	return 0;
}

