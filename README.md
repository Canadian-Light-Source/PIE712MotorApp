# PIE712 Motor Controller EPICS Support

This repository provides EPICS support for the Physik Instrumente E-712 motor controller, including IOC applications, parameter file handling, and integration with asyn and autosave modules. This driver exposes most if not all? of the capabilities of the E712 namely
waveform generator and data recorder control. This driver was adapted by Russ Berg (Canadian Lightsource) from and early generic GCS2 driver by Steffen Rau, and is in primary use at the 10ID1 STXM beamline at the CLS.

## Project Structure

- `PIE712Support/` - Core device support and controller logic
- `PIE712App/` - Application layer, including parameter file loading and IOC startup scripts
- `db/`, `dbd/` - Database and device definition files
- `iocBoot/` - IOC boot directories and startup scripts
- `paramFiles/` - Example parameter files for the controller
- `python/` - Python scripts for parameter file processing
- `configure/` - EPICS build configuration files
- `lib/`, `bin/` - Compiled libraries and binaries

## Building

To build the project, ensure you have EPICS Base and required modules installed, as well the parameter file loading makes use of python
so check the Makefiles for python header file include directories and set them to your locations.
Make the PIE712Support library first then make the app, so first

```sh
cd PIE712Support/src 
make
```
Then build the app
```sh
cd PIE712App/src 
make
cd PIE712App/Db
make 
```
This should create the binary and databases.

## Usage

1. Edit or create parameter files in `paramFiles/`.
2. Configure IOC startup scripts in `iocBoot/`.
3. Start the IOC using the appropriate boot script.

## Parameter File Loading

Parameter files (`*.pam`) are processed by the Python script `python/e712_load_parm_file.py`. The IOC state machine (`load_param_file.st`) calls this script to convert parameter files into controller commands, this is intended to be called from the st file..

## Operator screens

The `opi` directory contains a script and .adl files to look at an E712 with 2 channels and can be loaded by running the script.

## License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE](LICENSE) file for details.


