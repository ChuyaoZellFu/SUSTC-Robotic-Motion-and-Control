// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ActuatorCustomData = require('./ActuatorCustomData.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');

//-----------------------------------------------------------

class BaseCyclic_CustomData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frame_id = null;
      this.custom_data_0 = null;
      this.custom_data_1 = null;
      this.custom_data_2 = null;
      this.custom_data_3 = null;
      this.custom_data_4 = null;
      this.custom_data_5 = null;
      this.custom_data_6 = null;
      this.custom_data_7 = null;
      this.actuators_custom_data = null;
      this.interconnect_custom_data = null;
    }
    else {
      if (initObj.hasOwnProperty('frame_id')) {
        this.frame_id = initObj.frame_id
      }
      else {
        this.frame_id = 0;
      }
      if (initObj.hasOwnProperty('custom_data_0')) {
        this.custom_data_0 = initObj.custom_data_0
      }
      else {
        this.custom_data_0 = 0;
      }
      if (initObj.hasOwnProperty('custom_data_1')) {
        this.custom_data_1 = initObj.custom_data_1
      }
      else {
        this.custom_data_1 = 0;
      }
      if (initObj.hasOwnProperty('custom_data_2')) {
        this.custom_data_2 = initObj.custom_data_2
      }
      else {
        this.custom_data_2 = 0;
      }
      if (initObj.hasOwnProperty('custom_data_3')) {
        this.custom_data_3 = initObj.custom_data_3
      }
      else {
        this.custom_data_3 = 0;
      }
      if (initObj.hasOwnProperty('custom_data_4')) {
        this.custom_data_4 = initObj.custom_data_4
      }
      else {
        this.custom_data_4 = 0;
      }
      if (initObj.hasOwnProperty('custom_data_5')) {
        this.custom_data_5 = initObj.custom_data_5
      }
      else {
        this.custom_data_5 = 0;
      }
      if (initObj.hasOwnProperty('custom_data_6')) {
        this.custom_data_6 = initObj.custom_data_6
      }
      else {
        this.custom_data_6 = 0;
      }
      if (initObj.hasOwnProperty('custom_data_7')) {
        this.custom_data_7 = initObj.custom_data_7
      }
      else {
        this.custom_data_7 = 0;
      }
      if (initObj.hasOwnProperty('actuators_custom_data')) {
        this.actuators_custom_data = initObj.actuators_custom_data
      }
      else {
        this.actuators_custom_data = [];
      }
      if (initObj.hasOwnProperty('interconnect_custom_data')) {
        this.interconnect_custom_data = initObj.interconnect_custom_data
      }
      else {
        this.interconnect_custom_data = new InterconnectCyclic_CustomData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BaseCyclic_CustomData
    // Serialize message field [frame_id]
    bufferOffset = _serializer.uint32(obj.frame_id, buffer, bufferOffset);
    // Serialize message field [custom_data_0]
    bufferOffset = _serializer.uint32(obj.custom_data_0, buffer, bufferOffset);
    // Serialize message field [custom_data_1]
    bufferOffset = _serializer.uint32(obj.custom_data_1, buffer, bufferOffset);
    // Serialize message field [custom_data_2]
    bufferOffset = _serializer.uint32(obj.custom_data_2, buffer, bufferOffset);
    // Serialize message field [custom_data_3]
    bufferOffset = _serializer.uint32(obj.custom_data_3, buffer, bufferOffset);
    // Serialize message field [custom_data_4]
    bufferOffset = _serializer.uint32(obj.custom_data_4, buffer, bufferOffset);
    // Serialize message field [custom_data_5]
    bufferOffset = _serializer.uint32(obj.custom_data_5, buffer, bufferOffset);
    // Serialize message field [custom_data_6]
    bufferOffset = _serializer.uint32(obj.custom_data_6, buffer, bufferOffset);
    // Serialize message field [custom_data_7]
    bufferOffset = _serializer.uint32(obj.custom_data_7, buffer, bufferOffset);
    // Serialize message field [actuators_custom_data]
    // Serialize the length for message field [actuators_custom_data]
    bufferOffset = _serializer.uint32(obj.actuators_custom_data.length, buffer, bufferOffset);
    obj.actuators_custom_data.forEach((val) => {
      bufferOffset = ActuatorCustomData.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [interconnect_custom_data]
    bufferOffset = InterconnectCyclic_CustomData.serialize(obj.interconnect_custom_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BaseCyclic_CustomData
    let len;
    let data = new BaseCyclic_CustomData(null);
    // Deserialize message field [frame_id]
    data.frame_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [custom_data_0]
    data.custom_data_0 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [custom_data_1]
    data.custom_data_1 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [custom_data_2]
    data.custom_data_2 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [custom_data_3]
    data.custom_data_3 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [custom_data_4]
    data.custom_data_4 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [custom_data_5]
    data.custom_data_5 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [custom_data_6]
    data.custom_data_6 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [custom_data_7]
    data.custom_data_7 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [actuators_custom_data]
    // Deserialize array length for message field [actuators_custom_data]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.actuators_custom_data = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.actuators_custom_data[i] = ActuatorCustomData.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [interconnect_custom_data]
    data.interconnect_custom_data = InterconnectCyclic_CustomData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 68 * object.actuators_custom_data.length;
    length += InterconnectCyclic_CustomData.getMessageSize(object.interconnect_custom_data);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BaseCyclic_CustomData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de86e2c250eb6e05b62cbbd55c2a087b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 frame_id
    uint32 custom_data_0
    uint32 custom_data_1
    uint32 custom_data_2
    uint32 custom_data_3
    uint32 custom_data_4
    uint32 custom_data_5
    uint32 custom_data_6
    uint32 custom_data_7
    ActuatorCustomData[] actuators_custom_data
    InterconnectCyclic_CustomData interconnect_custom_data
    ================================================================================
    MSG: kortex_driver/ActuatorCustomData
    
    uint32 command_id
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
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_CustomData
    
    InterconnectCyclic_MessageId custom_data_id
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
    InterconnectCyclic_CustomData_tool_customData oneof_tool_customData
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_MessageId
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/InterconnectCyclic_CustomData_tool_customData
    
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
    const resolved = new BaseCyclic_CustomData(null);
    if (msg.frame_id !== undefined) {
      resolved.frame_id = msg.frame_id;
    }
    else {
      resolved.frame_id = 0
    }

    if (msg.custom_data_0 !== undefined) {
      resolved.custom_data_0 = msg.custom_data_0;
    }
    else {
      resolved.custom_data_0 = 0
    }

    if (msg.custom_data_1 !== undefined) {
      resolved.custom_data_1 = msg.custom_data_1;
    }
    else {
      resolved.custom_data_1 = 0
    }

    if (msg.custom_data_2 !== undefined) {
      resolved.custom_data_2 = msg.custom_data_2;
    }
    else {
      resolved.custom_data_2 = 0
    }

    if (msg.custom_data_3 !== undefined) {
      resolved.custom_data_3 = msg.custom_data_3;
    }
    else {
      resolved.custom_data_3 = 0
    }

    if (msg.custom_data_4 !== undefined) {
      resolved.custom_data_4 = msg.custom_data_4;
    }
    else {
      resolved.custom_data_4 = 0
    }

    if (msg.custom_data_5 !== undefined) {
      resolved.custom_data_5 = msg.custom_data_5;
    }
    else {
      resolved.custom_data_5 = 0
    }

    if (msg.custom_data_6 !== undefined) {
      resolved.custom_data_6 = msg.custom_data_6;
    }
    else {
      resolved.custom_data_6 = 0
    }

    if (msg.custom_data_7 !== undefined) {
      resolved.custom_data_7 = msg.custom_data_7;
    }
    else {
      resolved.custom_data_7 = 0
    }

    if (msg.actuators_custom_data !== undefined) {
      resolved.actuators_custom_data = new Array(msg.actuators_custom_data.length);
      for (let i = 0; i < resolved.actuators_custom_data.length; ++i) {
        resolved.actuators_custom_data[i] = ActuatorCustomData.Resolve(msg.actuators_custom_data[i]);
      }
    }
    else {
      resolved.actuators_custom_data = []
    }

    if (msg.interconnect_custom_data !== undefined) {
      resolved.interconnect_custom_data = InterconnectCyclic_CustomData.Resolve(msg.interconnect_custom_data)
    }
    else {
      resolved.interconnect_custom_data = new InterconnectCyclic_CustomData()
    }

    return resolved;
    }
};

module.exports = BaseCyclic_CustomData;
