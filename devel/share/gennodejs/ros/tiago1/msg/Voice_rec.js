// Auto-generated. Do not edit!

// (in-package tiago1.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Voice_rec {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id_client = null;
      this.list_of_orders = null;
    }
    else {
      if (initObj.hasOwnProperty('id_client')) {
        this.id_client = initObj.id_client
      }
      else {
        this.id_client = 0;
      }
      if (initObj.hasOwnProperty('list_of_orders')) {
        this.list_of_orders = initObj.list_of_orders
      }
      else {
        this.list_of_orders = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Voice_rec
    // Serialize message field [id_client]
    bufferOffset = _serializer.int32(obj.id_client, buffer, bufferOffset);
    // Serialize message field [list_of_orders]
    bufferOffset = _arraySerializer.string(obj.list_of_orders, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Voice_rec
    let len;
    let data = new Voice_rec(null);
    // Deserialize message field [id_client]
    data.id_client = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [list_of_orders]
    data.list_of_orders = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.list_of_orders.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tiago1/Voice_rec';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e4a95f50c96541db33e3cde77c3a65f4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 id_client
    string[] list_of_orders
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Voice_rec(null);
    if (msg.id_client !== undefined) {
      resolved.id_client = msg.id_client;
    }
    else {
      resolved.id_client = 0
    }

    if (msg.list_of_orders !== undefined) {
      resolved.list_of_orders = msg.list_of_orders;
    }
    else {
      resolved.list_of_orders = []
    }

    return resolved;
    }
};

module.exports = Voice_rec;
