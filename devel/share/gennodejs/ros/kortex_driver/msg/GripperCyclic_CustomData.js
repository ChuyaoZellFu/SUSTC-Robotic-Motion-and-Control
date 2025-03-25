// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let CustomDataUnit = require('./CustomDataUnit.js');

//-----------------------------------------------------------

class GripperCyclic_CustomData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.custom_data_id = null;
      this.gripper_custom_data = null;
      this.motor_custom_data = null;
    }
    else {
      if (initObj.hasOwnProperty('custom_data_id')) {
        this.custom_data_id = initObj.custom_data_id
      }
      else {
        this.custom_data_id = new GripperCyclic_MessageId();
      }
      if (initObj.hasOwnProperty('gripper_custom_data')) {
        this.gripper_custom_data = initObj.gripper_custom_data
      }
      else {
        this.gripper_custom_data = new CustomDataUnit();
      }
      if (initObj.hasOwnProperty('motor_custom_data')) {
        this.motor_custom_data = initObj.motor_custom_data
      }
      else {
        this.motor_custom_data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperCyclic_CustomData
    // Serialize message field [custom_data_id]
    bufferOffset = GripperCyclic_MessageId.serialize(obj.custom_data_id, buffer, bufferOffset);
    // Serialize message field [gripper_custom_data]
    bufferOffset = CustomDataUnit.serialize(obj.gripper_custom_data, buffer, bufferOffset);
    // Serialize message field [motor_custom_data]
    // Serialize the length for message field [motor_custom_data]
    bufferOffset = _serializer.uint32(obj.motor_custom_data.length, buffer, bufferOffset);
    obj.motor_custom_data.forEach((val) => {
      bufferOffset = CustomDataUnit.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperCyclic_CustomData
    let len;
    let data = new GripperCyclic_CustomData(null);
    // Deserialize message field [custom_data_id]
    data.custom_data_id = GripperCyclic_MessageId.deserialize(buffer, bufferOffset);
    // Deserialize message field [gripper_custom_data]
    data.gripper_custom_data = CustomDataUnit.deserialize(buffer, bufferOffset);
    // Deserialize message field [motor_custom_data]
    // Deserialize array length for message field [motor_custom_data]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.motor_custom_data = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.motor_custom_data[i] = CustomDataUnit.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 64 * object.motor_custom_data.length;
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GripperCyclic_CustomData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ccb37c7f71598724d847a97b3a0c62f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    GripperCyclic_MessageId custom_data_id
    CustomDataUnit gripper_custom_data
    CustomDataUnit[] motor_custom_data
    ================================================================================
    MSG: kortex_driver/GripperCyclic_MessageId
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/CustomDataUnit
    
    uint32 custom_data_0
    uint32 custom_data_1
    uint32 custom_data_2
    uint32 custom_data_3
    uint32 custom_data_4
    uint32 custom_data_5
    uint32 custom_data_6
    uint32 custom_data_7
    uint32 custom_data_8
    uint32 custom_data_9
    uint32 custom_data_10
    uint32 custom_data_11
    uint32 custom_data_12
    uint32 custom_data_13
    uint32 custom_data_14
    uint32 custom_data_15
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperCyclic_CustomData(null);
    if (msg.custom_data_id !== undefined) {
      resolved.custom_data_id = GripperCyclic_MessageId.Resolve(msg.custom_data_id)
    }
    else {
      resolved.custom_data_id = new GripperCyclic_MessageId()
    }

    if (msg.gripper_custom_data !== undefined) {
      resolved.gripper_custom_data = CustomDataUnit.Resolve(msg.gripper_custom_data)
    }
    else {
      resolved.gripper_custom_data = new CustomDataUnit()
    }

    if (msg.motor_custom_data !== undefined) {
      resolved.motor_custom_data = new Array(msg.motor_custom_data.length);
      for (let i = 0; i < resolved.motor_custom_data.length; ++i) {
        resolved.motor_custom_data[i] = CustomDataUnit.Resolve(msg.motor_custom_data[i]);
      }
    }
    else {
      resolved.motor_custom_data = []
    }

    return resolved;
    }
};

module.exports = GripperCyclic_CustomData;
