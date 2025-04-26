// Auto-generated. Do not edit!

// (in-package tiago1.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class robotstatedecisionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state_input = null;
      this.robot_id = null;
    }
    else {
      if (initObj.hasOwnProperty('state_input')) {
        this.state_input = initObj.state_input
      }
      else {
        this.state_input = '';
      }
      if (initObj.hasOwnProperty('robot_id')) {
        this.robot_id = initObj.robot_id
      }
      else {
        this.robot_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robotstatedecisionRequest
    // Serialize message field [state_input]
    bufferOffset = _serializer.string(obj.state_input, buffer, bufferOffset);
    // Serialize message field [robot_id]
    bufferOffset = _serializer.string(obj.robot_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robotstatedecisionRequest
    let len;
    let data = new robotstatedecisionRequest(null);
    // Deserialize message field [state_input]
    data.state_input = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [robot_id]
    data.robot_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.state_input);
    length += _getByteLength(object.robot_id);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tiago1/robotstatedecisionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '474dfee8de6c5b142980ea7175d700ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string state_input 
    string robot_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new robotstatedecisionRequest(null);
    if (msg.state_input !== undefined) {
      resolved.state_input = msg.state_input;
    }
    else {
      resolved.state_input = ''
    }

    if (msg.robot_id !== undefined) {
      resolved.robot_id = msg.robot_id;
    }
    else {
      resolved.robot_id = ''
    }

    return resolved;
    }
};

class robotstatedecisionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state_output = null;
      this.id_client = null;
      this.order = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('state_output')) {
        this.state_output = initObj.state_output
      }
      else {
        this.state_output = '';
      }
      if (initObj.hasOwnProperty('id_client')) {
        this.id_client = initObj.id_client
      }
      else {
        this.id_client = 0;
      }
      if (initObj.hasOwnProperty('order')) {
        this.order = initObj.order
      }
      else {
        this.order = [];
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robotstatedecisionResponse
    // Serialize message field [state_output]
    bufferOffset = _serializer.string(obj.state_output, buffer, bufferOffset);
    // Serialize message field [id_client]
    bufferOffset = _serializer.int32(obj.id_client, buffer, bufferOffset);
    // Serialize message field [order]
    bufferOffset = _arraySerializer.string(obj.order, buffer, bufferOffset, null);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robotstatedecisionResponse
    let len;
    let data = new robotstatedecisionResponse(null);
    // Deserialize message field [state_output]
    data.state_output = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [id_client]
    data.id_client = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [order]
    data.order = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.state_output);
    object.order.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tiago1/robotstatedecisionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '52bb16fdddb35bef99f76c3abb84b931';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string state_output
    int32 id_client
    string[] order
    bool success
    
    
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new robotstatedecisionResponse(null);
    if (msg.state_output !== undefined) {
      resolved.state_output = msg.state_output;
    }
    else {
      resolved.state_output = ''
    }

    if (msg.id_client !== undefined) {
      resolved.id_client = msg.id_client;
    }
    else {
      resolved.id_client = 0
    }

    if (msg.order !== undefined) {
      resolved.order = msg.order;
    }
    else {
      resolved.order = []
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: robotstatedecisionRequest,
  Response: robotstatedecisionResponse,
  md5sum() { return '33ec071b72a9a044f86da64824d0e481'; },
  datatype() { return 'tiago1/robotstatedecision'; }
};
