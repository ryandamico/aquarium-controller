# Aquarium Controller - Historical Code Snapshots

This repository contains the historical evolution of a Particle IoT aquarium automation system, tracked from Particle Build web IDE backups (September 2022 - October 2025).

## System Overview

A comprehensive aquarium automation platform featuring:
- Automatic water top-off with dual sensors
- Temperature monitoring and heater control
- Active cooling system with precision control
- CO2 injection management
- Automated water changes via mixing station
- Dosing pump control
- Flow rate monitoring
- Water leak detection
- Remote monitoring via Particle Cloud

## Repository History

This repository was created from 10 backup snapshots spanning 3+ years:

1. **Sep 2022** - Initial implementation (basic water topoff, heater control)
2. **Oct 2023** - Major expansion (IO boards, mixing station)
3. **Mar 2025** - Siphon cleaning mode and refinements
4. **Jun 2025** - Dosing pump algorithm overhaul
5. **Jul 2025** - Complete cooling system implementation
6. **Oct 2025** - Production hardening with safety limits

Each commit includes detailed AI-generated summaries of changes.

## Adding New Backups

Use the included `commit-backup.sh` script to add new Particle Build backups:

### Quick Usage

```bash
# With a commit message
./commit-backup.sh path/to/backup.zip -m "Fix cooling pump timing"

# Auto-generate message with Claude Code
./commit-backup.sh path/to/backup.zip --auto-message

# Suggest mode (stage changes but don't commit)
./commit-backup.sh path/to/backup.zip --suggest
```

### Workflow Examples

**Example 1: Manual commit message**
```bash
cd git-repo-code-snapshots
./commit-backup.sh "../Code backups/Aquarium_Controller (7).zip" -m "Add pH sensor support"
git push
```

**Example 2: Claude Code analysis**
```bash
cd git-repo-code-snapshots
./commit-backup.sh "../Code backups/Aquarium_Controller (7).zip" --suggest

# Then in Claude Code, ask:
# "Review the staged changes and suggest a commit message following the repo style"

# After Claude suggests a message:
git commit -m "Paste Claude's suggested message here"
git push
```

**Example 3: Fully automated (requires Claude Code in terminal)**
```bash
./commit-backup.sh backup.zip --auto-message
# Follow the prompts to get Claude's analysis
# Then commit with the suggested message
```

## Script Details

The `commit-backup.sh` script:
- Extracts the backup zip to the repository
- Stages all changes
- Shows statistics about what changed
- Commits with your message or helps generate one via Claude Code
- Preserves the `.git` directory and README

### Options

- `-m, --message MSG` - Commit with the specified message
- `--auto-message` - Get help from Claude Code to analyze and suggest a message
- `--suggest` - Stage changes and show diff, but don't commit (for review)
- `-h, --help` - Show help message

## Hardware Configuration

The system is designed for Particle IoT devices (Photon/Argon/Boron) with:
- DS18B20 temperature sensors
- Capacitive and optical water level sensors
- Flow rate sensor
- MCP23017 I2C IO expanders
- Multiple relay modules
- PWM-controlled pumps
- RGB LCD display (Waveshare 1602)

## Project Structure

- `aquarium-controller.ino` - Main controller code
- `CO2BubbleSensor_v3.h` - CO2 bubble counting and control
- `CoolingPump.h` - Active cooling system manager
- `MixingStationIO.h` - Water preparation system
- `FlowRateSensor.h` - Flow rate monitoring
- `SimpleScheduler.h` - Task scheduling
- Various utility and hardware abstraction modules

## Safety Features

- Hardware watchdog (5-minute timeout)
- Water leak detection with automatic shutoff
- Multiple temperature safety limits
- Flow rate monitoring with heater interlock
- 12-hour cooling system runtime limit
- Automatic safe mode on startup issues

## Cloud Integration

- Particle Cloud functions for remote control
- Google Sheets logging
- Push notifications for alerts
- Real-time sensor data publishing

## License

AI-generated summary from code snapshots - check individual file headers for specifics.
