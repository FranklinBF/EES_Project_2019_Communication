# Simple discover devices over BT
import bluetooth

# Find devices
nearby_devices = bluetooth.discover_devices(lookup_names=True)
print("found %d devices" % len(nearby_devices))

# Print each found device with addr and name
for addr, name in nearby_devices:
    print("  %s - %s" % (addr, name))
