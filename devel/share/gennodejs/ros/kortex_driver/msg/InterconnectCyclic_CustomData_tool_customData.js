// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');

//-----------------------------------------------------------

class InterconnectCyclic_CustomData_tool_customData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gripper_custom_data = null;
    }
    else {
      if (initObj.hasOwnProperty('gripper_custom_data')) {
        this.gripper_custom_data = initObj.gripper_custom_data
      }
      else {
        this.gripper_custom_data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InterconnectCyclic_CustomData_tool_customData
    // Serialize message field [gripper_custom_data]
    // Serialize the length for message field [gripper_custom_data]
    bufferOffset = _serializer.uint32(obj.gripper_custom_data.length, buffer, bufferOffset);
    obj.gripper_custom_data.forEach((val) => {
      bufferOffset = GripperCyclic_CustomData.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InterconnectCyclic_CustomData_tool_customData
    let len;
    let data = new InterconnectCyclic_CustomData_tool_customData(null);
    // Deserialize message field [gripper_custom_data]
    // Deserialize array length for message field [gripper_custom_data]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.gripper_custom_data = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.gripper_custom_data[i] = GripperCyclic_CustomData.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.gripper_custom_data.forEach((val) => {
      length += GripperCyclic_CustomData.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/InterconnectCyclic_CustomData_tool_customData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '08300f61864775cfea0ba45676ef4872';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    GripperCyclic_CustomData[] gripper_custom_data
    ================================================================================
    MSG: kortex_driver/GripperCyclic_CustomData
    
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
    const resolved = new InterconnectCyclic_CustomData_tool_customData(null);
    if (msg.gripper_custom_data !== undefined) {
      resolved.gripper_custom_data = new Array(msg.gripper_custom_data.length);
      for (let i = 0; i < resolved.gripper_custom_data.length; ++i) {
        resolved.gripper_custom_data[i] = GripperCyclic_CustomData.Resolve(msg.gripper_custom_data[i]);
      }
    }
    else {
      resolved.gripper_custom_data = []
    }

    return resolved;
    }
};

module.exports = InterconnectCyclic_CustomData_tool_customData;
