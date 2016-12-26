# smbus
Go bindings for the System Management Bus (SMBus) kernel interface
This package provides simple bindings for the SMBus interfaces provided by the i2c-dev driver.
I wrote this based on https://github.com/corrupt/go-smbus

**This code is largely untested. I'll happily accept a pull request for any bugs you might find**

## Installation

    go get github.com/torukita/smbus

## Usage

Create an interface instance of `SMBus` using the single method for each bus id. It takes the interface index which is the enumerated device index. If your I2C device is `/dev/i2c-1`, your index is 1.


```go
smb := smbus.GetSMBus(1)
```

You can now use the SMBus API to write to and read from the bus. All methods evaluate `errno` and return a go error accordingly. Block read/write methods also return the number of read/written bytes.

```go
addr := 0x77
cmd := 0xD0
val := 0x10
err := smb.WriteByteData(addr, cmd, val)

buf := make ([]byte, 4)
i, err := smb.ReadI22cBlock_data(0x77, 0xD1, buf)
if err != nil {
    fmt.Println(err)              
    //error handling
}
if i != len(buf) {
    //error handling
}
```
