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
    }
    else {
      if (initObj.hasOwnProperty('order')) {
        this.order = initObj.order
      }
      else {
        this.order = new Voice_rec();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type send_orderRequest
    // Serialize message field [order]
    bufferOffset = Voice_rec.serialize(obj.order, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type send_orderRequest
    let len;
    let data = new send_orderRequest(null);
    // Deserialize message field [order]
    data.order = Voice_rec.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Voice_rec.getMessageSize(object.order);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tiago1/send_orderRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c4f6fe1e5b6183c9cba764c155d0803b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    tiago1/Voice_rec order
    
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
  md5sum() { return '741481bb0a1ea48957f004257c6bcdd4'; },
  datatype() { return 'tiago1/send_order'; }
};
