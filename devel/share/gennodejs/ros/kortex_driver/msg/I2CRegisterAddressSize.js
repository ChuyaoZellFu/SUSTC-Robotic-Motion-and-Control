// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class I2CRegisterAddressSize {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type I2CRegisterAddressSize
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CRegisterAddressSize
    let len;
    let data = new I2CRegisterAddressSize(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/I2CRegisterAddressSize';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '517dd96a7b250066663891b1fad6bd8a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED = 0
    
    uint32 I2C_REGISTER_ADDRESS_SIZE_8_BITS = 1
    
    uint32 I2C_REGISTER_ADDRESS_SIZE_16_BITS = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new I2CRegisterAddressSize(null);
    return resolved;
    }
};

// Constants for message
I2CRegisterAddressSize.Constants = {
  I2C_REGISTER_ADDRESS_SIZE_UNSPECIFIED: 0,
  I2C_REGISTER_ADDRESS_SIZE_8_BITS: 1,
  I2C_REGISTER_ADDRESS_SIZE_16_BITS: 2,
}

module.exports = I2CRegisterAddressSize;
