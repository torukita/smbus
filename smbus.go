package smbus
/*
	Package smbus provides go bindings for the SMBus (System Management Bus) kernel interface
	SMBus is a subset of i2c suitable for a large number of devices
	Original domentation : https://www.kernel.org/doc/Documentation/i2c/smbus-protocol
*/

/*
#include "i2c-dev.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
*/
import "C"

import (
	"fmt"
	"os"
	"syscall"
	"unsafe"
	"sync"
)

const (
	i2c_SLAVE = 0x0703
)

type SMBus interface {
	WriteQuick(addr byte, value byte) (err error)
	ReadByte(addr byte) (value byte, err error)
	WriteByte(addr byte, value byte) (err error)
	ReadByteData(addr byte, cmd byte) (value byte, err error)
	WriteByteData(addr byte, cmd byte, value byte) (err error)
	ReadWordData(addr byte, cmd byte) (value uint16, err error)
	WriteWordData(addr byte, cmd byte, value uint16) (err error)
	ProcessCall(addr byte, cmd byte, value uint16) (uint16, error)
	ReadBlockData(addr byte, cmd byte, buf []byte) (int, error)
	WriteBlockData(addr byte, cmd byte, buf []byte) (int, error)
	ReadI2cBlockData(addr byte, cmd byte, buf []byte) (int, error)
	WriteI2cBlockData(addr byte, cmd byte, buf []byte) (int, error)
	BlockProcessCall(addr byte, cmd byte, buf []byte) ([]byte, error)
}

type smbus struct {
	bus        uint8         //bus id
	mutex      sync.Mutex
	file       *os.File     //Set when esecutig method
	cachedAddr byte         //Set when esecutig method
	opened     bool         //Status if file is opened
}

// Singleton structure
var busMap map[uint8]*smbus = map[uint8]*smbus{}
var mutex sync.Mutex
func GetSMBus(busID uint8) SMBus {
	if bus, ok := busMap[busID]; ok {
		return bus
	}
	mutex.Lock()
	defer mutex.Unlock()
	bus := &smbus{bus: busID}
	busMap[busID] = bus
	return bus
}

func (smb *smbus) open() error {
	if smb.opened {
		return nil
	}
	path := fmt.Sprintf("/dev/i2c-%d", smb.bus)
	f, err := os.OpenFile(path, os.O_RDWR, os.ModeDevice)
	if err != nil {
		return err
	}
	smb.file = f
	smb.opened = true
	return nil
}

// Set the i2c address in bus
// Should lock before calling this func
func (smb *smbus) setAddr(addr byte) error {
	if smb.cachedAddr != addr {
		if err := ioctl(smb.file.Fd(), i2c_SLAVE, uintptr(addr)); err != nil {
			return err
		}
		smb.cachedAddr = addr
	}
	return nil
}

func ioctl(fd, cmd, arg uintptr) error {
	_, _, errno := syscall.Syscall6(syscall.SYS_IOCTL, fd, cmd, arg, 0, 0, 0)
	if errno != 0 {
		return errno
	}
	return nil
}

// Sends a single bit to the device, at the place of the Rd/Wr bit.
func (smb *smbus) WriteQuick(addr, value byte) error {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return err
	}
	if err := smb.setAddr(addr); err != nil {
		return err
	}
	_, err := C.i2c_smbus_write_quick(C.int(smb.file.Fd()), C.__u8(value))
	return err
}

// Reads a single byte from a device, without specifying a device
// register. Some devices are so simple that this interface is enough;
// for others, it is a shorthand if you want to read the same register
// as in the previous smbus command.
func (smb *smbus) ReadByte(addr byte) (byte, error) {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return 0, err
	}
	if err := smb.setAddr(addr); err != nil {
		return 0, err
	}
	ret, err := C.i2c_smbus_read_byte(C.int(smb.file.Fd()))
	if err != nil {
		ret = 0
	}
	return byte(ret & 0x0FF), err
}

// This operation is the reverse of Receive Byte: it sends a single
// byte to a device. See Receive Byte for more information.
func (smb *smbus) WriteByte(addr, value byte) error {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return err
	}
	if err := smb.setAddr(addr); err != nil {
		return err
	}
	_, err := C.i2c_smbus_write_byte(C.int(smb.file.Fd()), C.__u8(value))
	return err
}

// Reads a single byte from a device, from a designated register.
// The register is specified through the cmd byte
func (smb *smbus) ReadByteData(addr, cmd byte) (byte, error) {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return 0, err
	}
	if err := smb.setAddr(addr); err != nil {
		return 0, err
	}
	ret, err := C.i2c_smbus_read_byte_data(C.int(smb.file.Fd()), C.__u8(cmd))
	if err != nil {
		ret = 0
	}
	return byte(ret & 0x0FF), err
}

// Writes a single byte to a device, to a designated register. The
// register is specified through the cmd byte. This is the opposite
// of the Read Byte operation.
func (smb *smbus) WriteByteData(addr, cmd, value byte) error {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return err
	}
	if err := smb.setAddr(addr); err != nil {
		return err
	}
	_, err := C.i2c_smbus_write_byte_data(C.int(smb.file.Fd()), C.__u8(cmd), C.__u8(value))
	return err
}

// This operation is very like Read Byte; again, data is read from a
// device, from a designated register that is specified through the cmd
// byte. But this time, the data is a complete word (16 bits).
func (smb *smbus) ReadWordData(addr, cmd byte) (uint16, error) {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return 0, err
	}
	if err := smb.setAddr(addr); err != nil {
		return 0, err
	}
	ret, err := C.i2c_smbus_read_word_data(C.int(smb.file.Fd()), C.__u8(cmd))
	if err != nil {
		ret = 0
	}
	return uint16(ret & 0x0FFFF), err
}

// This is the opposite of the Read Word operation. 16 bits
// of data is written to a device, to the designated register that is
// specified through the cmd byte.
func (smb *smbus) WriteWordData(addr, cmd byte, value uint16) error {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return err
	}
	if err := smb.setAddr(addr); err != nil {
		return err
	}
	_, err := C.i2c_smbus_write_word_data(C.int(smb.file.Fd()), C.__u8(cmd), C.__u16(value))
	return err
}

// This command selects a device register (through the cmd byte), sends
// 16 bits of data to it, and reads 16 bits of data in return.
func (smb *smbus) ProcessCall(addr, cmd byte, value uint16) (uint16, error) {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return 0, err
	}
	if err := smb.setAddr(addr); err != nil {
		return 0, err
	}
	ret, err := C.i2c_smbus_process_call(C.int(smb.file.Fd()), C.__u8(cmd), C.__u16(value))
	if err != nil {
		ret = 0
	}
	return uint16(ret & 0x0FFFF), err
}

// This command reads a block of up to 32 bytes from a device, from a
// designated register that is specified through the cmd byte. The amount
// of data in byte is specified by the length of the buf slice.
// To read 4 bytes of data, pass a slice created like this: make([]byte, 4)
func (smb *smbus) ReadBlockData(addr, cmd byte, buf []byte) (int, error) {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return 0, err
	}
	if err := smb.setAddr(addr); err != nil {
		return 0, err
	}
	ret, err := C.i2c_smbus_read_block_data(
		C.int(smb.file.Fd()),
		C.__u8(cmd),
		(*C.__u8)(unsafe.Pointer(&buf[0])),
	)
	return int(ret), err
}

// The opposite of the Block Read command, this writes up to 32 bytes to
// a device, to a designated register that is specified through the
// cmd byte. The amount of data is specified by the lengts of buf.
func (smb *smbus) WriteBlockData(addr, cmd byte, buf []byte) (int, error) {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return 0, err
	}
	if err := smb.setAddr(addr); err != nil {
		return 0, err
	}
	ret, err := C.i2c_smbus_write_block_data(C.int(smb.file.Fd()), C.__u8(cmd), C.__u8(len(buf)), ((*C.__u8)(&buf[0])))
	return int(ret), err
}

// Block read method for devices without smbus support. Uses plain i2c interface
func (smb *smbus) ReadI2cBlockData(addr, cmd byte, buf []byte) (int, error) {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return 0, err
	}
	if err := smb.setAddr(addr); err != nil {
		return 0, err
	}
	ret, err := C.i2c_smbus_read_i2c_block_data(C.int(smb.file.Fd()), C.__u8(cmd), C.__u8(len(buf)), ((*C.__u8)(&buf[0])))
	return int(ret), err
}

// Block write method for devices without smbus support. Uses plain i2c interface
func (smb *smbus) WriteI2cBlockData(addr, cmd byte, buf []byte) (int, error) {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return 0, err
	}
	if err := smb.setAddr(addr); err != nil {
		return 0, err
	}
	ret, err := C.i2c_smbus_write_i2c_block_data(C.int(smb.file.Fd()), C.__u8(cmd), C.__u8(len(buf)), ((*C.__u8)(&buf[0])))
	return int(ret), err
}

// This command selects a device register (through the cmd byte), sends
// 1 to 31 bytes of data to it, and reads 1 to 31 bytes of data in return.
func (smb *smbus) BlockProcessCall(addr, cmd byte, buf []byte) ([]byte, error) {
	smb.mutex.Lock()
	defer smb.mutex.Unlock()
	if err := smb.open(); err != nil {
		return nil, err
	}
	if err := smb.setAddr(addr); err != nil {
		return nil, err
	}
	ret, err := C.i2c_smbus_block_process_call(C.int(smb.file.Fd()), C.__u8(cmd), C.__u8(len(buf)), ((*C.__u8)(&buf[0])))
	if err != nil {
		return nil, err
	} else {
		return buf[:ret], nil
	}
}


