#!/usr/bin/env python3
"""
VESC CAN SDK - GDB Debugging Script

This script runs the VESC monitor with GDB to get detailed stack traces
when segmentation faults occur.
"""

import os
import sys
import subprocess
import tempfile

def create_gdb_script():
    """Create a GDB script for debugging segfaults"""
    gdb_script = """
# GDB script for debugging VESC CAN SDK segfaults
set pagination off
set confirm off

# Set up signal handling
handle SIGSEGV stop print
handle SIGBUS stop print
handle SIGILL stop print
handle SIGFPE stop print

# Run the program
run

# If we hit a segfault, print detailed information
if $_siginfo
    echo \\n=== SEGMENTATION FAULT DETECTED ===\\n
    echo Signal: $_siginfo.si_signo\\n
    echo Fault address: $_siginfo.si_addr\\n
    
    # Print backtrace
    echo === BACKTRACE ===\\n
    bt full
    
    # Print registers
    echo \\n=== REGISTERS ===\\n
    info registers
    
    # Print memory around fault address
    echo \\n=== MEMORY AROUND FAULT ADDRESS ===\\n
    x/20x $_siginfo.si_addr-0x40
    
    # Print Python stack if available
    echo \\n=== PYTHON STACK ===\\n
    py-bt
    
    # Print current thread info
    echo \\n=== THREAD INFO ===\\n
    info threads
    thread apply all bt
    
    # Print loaded libraries
    echo \\n=== LOADED LIBRARIES ===\\n
    info shared
    
    # Print breakpoints
    echo \\n=== BREAKPOINTS ===\\n
    info breakpoints
else
    echo \\n=== PROGRAM COMPLETED SUCCESSFULLY ===\\n
endif

quit
"""
    return gdb_script

def run_with_gdb(python_script, args=None):
    """Run a Python script with GDB debugging"""
    if args is None:
        args = []
    
    # Create temporary GDB script
    with tempfile.NamedTemporaryFile(mode='w', suffix='.gdb', delete=False) as f:
        gdb_script_path = f.name
        f.write(create_gdb_script())
    
    try:
        # Build the command
        cmd = [
            'gdb',
            '--batch',
            '--command=' + gdb_script_path,
            '--args',
            'python3',
            python_script
        ] + args
        
        print("Running with GDB:")
        print(" ".join(cmd))
        print("=" * 60)
        
        # Run the command
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        # Print output
        if result.stdout:
            print("STDOUT:")
            print(result.stdout)
        
        if result.stderr:
            print("STDERR:")
            print(result.stderr)
        
        print(f"Exit code: {result.returncode}")
        
        return result.returncode == 0
        
    finally:
        # Clean up temporary file
        os.unlink(gdb_script_path)

def run_with_valgrind(python_script, args=None):
    """Run a Python script with Valgrind for memory debugging"""
    if args is None:
        args = []
    
    # Build the command
    cmd = [
        'valgrind',
        '--tool=memcheck',
        '--leak-check=full',
        '--show-leak-kinds=all',
        '--track-origins=yes',
        '--verbose',
        '--log-file=valgrind_output.txt',
        'python3',
        python_script
    ] + args
    
    print("Running with Valgrind:")
    print(" ".join(cmd))
    print("=" * 60)
    
    # Run the command
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    # Print output
    if result.stdout:
        print("STDOUT:")
        print(result.stdout)
    
    if result.stderr:
        print("STDERR:")
        print(result.stderr)
    
    print(f"Exit code: {result.returncode}")
    print("Valgrind output saved to: valgrind_output.txt")
    
    return result.returncode == 0

def run_with_address_sanitizer(python_script, args=None):
    """Run a Python script with AddressSanitizer"""
    if args is None:
        args = []
    
    # Set environment variables for AddressSanitizer
    env = os.environ.copy()
    env['ASAN_OPTIONS'] = 'abort_on_error=1:print_stats=1:print_legend=1'
    env['PYTHONMALLOC'] = 'malloc'
    
    # Build the command
    cmd = ['python3', python_script] + args
    
    print("Running with AddressSanitizer:")
    print(" ".join(cmd))
    print("Environment: ASAN_OPTIONS=" + env.get('ASAN_OPTIONS', ''))
    print("=" * 60)
    
    # Run the command
    result = subprocess.run(cmd, capture_output=True, text=True, env=env)
    
    # Print output
    if result.stdout:
        print("STDOUT:")
        print(result.stdout)
    
    if result.stderr:
        print("STDERR:")
        print(result.stderr)
    
    print(f"Exit code: {result.returncode}")
    
    return result.returncode == 0

def main():
    """Main function"""
    if len(sys.argv) < 2:
        print("Usage: python3 debug_with_gdb.py <script> [args...]")
        print("")
        print("Debugging options:")
        print("  --gdb          Run with GDB (default)")
        print("  --valgrind     Run with Valgrind")
        print("  --asan         Run with AddressSanitizer")
        print("")
        print("Examples:")
        print("  python3 debug_with_gdb.py vesc_monitor.py can0")
        print("  python3 debug_with_gdb.py --valgrind vesc_monitor.py can0 --debug")
        print("  python3 debug_with_gdb.py --asan vesc_monitor.py can0")
        return 1
    
    # Parse arguments
    script_args = sys.argv[1:]
    debug_method = 'gdb'
    
    if script_args[0] == '--gdb':
        debug_method = 'gdb'
        script_args = script_args[1:]
    elif script_args[0] == '--valgrind':
        debug_method = 'valgrind'
        script_args = script_args[1:]
    elif script_args[0] == '--asan':
        debug_method = 'asan'
        script_args = script_args[1:]
    
    if not script_args:
        print("ERROR: No script specified")
        return 1
    
    script_path = script_args[0]
    script_args = script_args[1:]
    
    # Check if script exists
    if not os.path.exists(script_path):
        print(f"ERROR: Script not found: {script_path}")
        return 1
    
    # Run with selected debugging method
    if debug_method == 'gdb':
        success = run_with_gdb(script_path, script_args)
    elif debug_method == 'valgrind':
        success = run_with_valgrind(script_path, script_args)
    elif debug_method == 'asan':
        success = run_with_address_sanitizer(script_path, script_args)
    else:
        print(f"ERROR: Unknown debugging method: {debug_method}")
        return 1
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main()) 