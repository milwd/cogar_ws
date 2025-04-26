// Auto-generated. Do not edit!

// (in-package tiago1.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Voice_rec = require('../msg/Voice_rec.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class send_orderRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.order = null;
      this.robot_id = null;
    }
    else {
      if (initObj.hasOwnProperty('order')) {
        this.order = initObj.order
      }
      else {
        this.order = new Voice_rec();
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
    // Serializes a message object of type send_orderRequest
    // Serialize message field [order]
    bufferOffset = Voice_rec.serialize(obj.order, buffer, bufferOffset);
    // Serialize message field [robot_id]
    bufferOffset = _serializer.string(obj.robot_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type send_orderRequest
    let len;
    let data = new send_orderRequest(null);
    // Deserialize message field [order]
    data.order = Voice_rec.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_id]
    data.robot_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Voice_rec.getMessageSize(object.order);
    length += _getByteLength(object.robot_id);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tiago1/send_orderRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd56ba497c9b776350b5159a4a79a8008';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    tiago1/Voice_rec order
    string robot_id
    
    ================================================================================
    MSG: tiago1/Voice_rec
    int32 id_client
    string[] list_of_orders
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new send_orderRequest(null);
    if (msg.order !== undefined) {
      resolved.order = Voice_rec.Resolve(msg.order)
    }
    else {
      resolved.order = new Voice_rec()
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

class send_orderResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type send_orderResponse
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type send_orderResponse
    let len;
    let data = new send_orderResponse(null);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tiago1/send_orderResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f003d6bcc824cbd51361d66d8e4f76c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string message
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new send_orderResponse(null);
    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: send_orderRequest,
  Response: send_orderResponse,
  md5sum() { return '54d692335fd48f1476d58fcf2615595d'; },
  datatype() { return 'tiago1/send_order'; }
};
