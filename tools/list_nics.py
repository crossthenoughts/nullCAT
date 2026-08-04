#!/usr/bin/env python3
"""
list_nics.py
============
Lists all network adapters on this Windows machine.
Run this to find the correct NIC name for host.json.

Usage:
    python list_nics.py

Requires: Python 3.6+ on Windows
"""

import subprocess
import sys

def list_nics_ipconfig():
    """Parse ipconfig /all for adapter names and descriptions."""
    try:
        result = subprocess.run(
            ["ipconfig", "/all"],
            capture_output=True, text=True, encoding="cp850"
        )
        output = result.stdout
        
        print("=" * 60)
        print("Network Adapters (from ipconfig /all)")
        print("=" * 60)
        print()
        print("Look for your EtherCAT NIC in this list.")
        print("Use the 'Description' value as 'nicName' in host.json.")
        print()
        
        lines = output.splitlines()
        in_adapter = False
        adapter_name = ""
        description = ""
        
        for line in lines:
            # New adapter section
            if "adapter" in line.lower() and line.endswith(":"):
                if adapter_name and description:
                    print(f"  Adapter:     {adapter_name}")
                    print(f"  Description: {description}")
                    print()
                adapter_name = line.strip().rstrip(":")
                description = ""
                in_adapter = True
            elif in_adapter and "Description" in line:
                description = line.split(":", 1)[1].strip()
        
        # Print last adapter
        if adapter_name and description:
            print(f"  Adapter:     {adapter_name}")
            print(f"  Description: {description}")
            print()
        
        print("=" * 60)
        print()
        print("Example host.json entry:")
        print('  "nicName": "Ethernet 2"')
        print()
        print("Windows: use the 'Description' value as nicName.")
        print("Linux/Pi: use the interface name instead (e.g. 'eth0').")
        
    except Exception as e:
        print(f"Error running ipconfig: {e}")
        print()
        print("Try running: ipconfig /all")
        print("and look for your EtherCAT NIC manually.")

if __name__ == "__main__":
    if sys.platform != "win32":
        print("This script is for Windows only.")
        print("On Linux: use 'ip link show' or 'ifconfig'")
        sys.exit(1)
    
    list_nics_ipconfig()
