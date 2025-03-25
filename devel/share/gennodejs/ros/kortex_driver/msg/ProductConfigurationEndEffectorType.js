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

class ProductConfigurationEndEffectorType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.end_effector_type = null;
    }
    else {
      if (initObj.hasOwnProperty('end_effector_type')) {
        this.end_effector_type = initObj.end_effector_type
      }
      else {
        this.end_effector_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ProductConfigurationEndEffectorType
    // Serialize message field [end_effector_type]
    bufferOffset = _serializer.uint32(obj.end_effector_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ProductConfigurationEndEffectorType
    let len;
    let data = new ProductConfigurationEndEffectorType(null);
    // Deserialize message field [end_effector_type]
    data.end_effector_type = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ProductConfigurationEndEffectorType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '471e42663382672f68e1e5c83df5fb95';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 end_effector_type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ProductConfigurationEndEffectorType(null);
    if (msg.end_effector_type !== undefined) {
      resolved.end_effector_type = msg.end_effector_type;
    }
    else {
      resolved.end_effector_type = 0
    }

    return resolved;
    }
};

module.exports = ProductConfigurationEndEffectorType;
