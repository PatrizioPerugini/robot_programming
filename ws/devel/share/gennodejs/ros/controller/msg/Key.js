// Auto-generated. Do not edit!

// (in-package controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Key {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.code = null;
      this.modifiers = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('code')) {
        this.code = initObj.code
      }
      else {
        this.code = 0;
      }
      if (initObj.hasOwnProperty('modifiers')) {
        this.modifiers = initObj.modifiers
      }
      else {
        this.modifiers = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Key
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [code]
    bufferOffset = _serializer.uint16(obj.code, buffer, bufferOffset);
    // Serialize message field [modifiers]
    bufferOffset = _serializer.uint16(obj.modifiers, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Key
    let len;
    let data = new Key(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [code]
    data.code = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [modifiers]
    data.modifiers = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'controller/Key';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5d57616b5631968b8f1d31d23c83281f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 KEY_UNKNOWN=0
    uint16 KEY_FIRST=0
    uint16 KEY_BACKSPACE=8
    uint16 KEY_TAB=9
    uint16 KEY_CLEAR=12
    uint16 KEY_RETURN=13
    uint16 KEY_PAUSE=19
    uint16 KEY_ESCAPE=27
    uint16 KEY_SPACE=32
    uint16 KEY_EXCLAIM=33
    uint16 KEY_QUOTEDBL=34
    uint16 KEY_HASH=35
    uint16 KEY_DOLLAR=36
    uint16 KEY_AMPERSAND=38
    uint16 KEY_QUOTE=39
    uint16 KEY_LEFTPAREN=40
    uint16 KEY_RIGHTPAREN=41
    uint16 KEY_ASTERISK=42
    uint16 KEY_PLUS=43
    uint16 KEY_COMMA=44
    uint16 KEY_MINUS=45
    uint16 KEY_PERIOD=46
    uint16 KEY_SLASH=47
    uint16 KEY_0=48
    uint16 KEY_1=49
    uint16 KEY_2=50
    uint16 KEY_3=51
    uint16 KEY_4=52
    uint16 KEY_5=53
    uint16 KEY_6=54
    uint16 KEY_7=55
    uint16 KEY_8=56
    uint16 KEY_9=57
    uint16 KEY_COLON=58
    uint16 KEY_SEMICOLON=59
    uint16 KEY_LESS=60
    uint16 KEY_EQUALS=61
    uint16 KEY_GREATER=62
    uint16 KEY_QUESTION=63
    uint16 KEY_AT=64
    uint16 KEY_LEFTBRACKET=91
    uint16 KEY_BACKSLASH=92
    uint16 KEY_RIGHTBRACKET=93
    uint16 KEY_CARET=94
    uint16 KEY_UNDERSCORE=95
    uint16 KEY_BACKQUOTE=96
    uint16 KEY_a=97
    uint16 KEY_b=98
    uint16 KEY_c=99
    uint16 KEY_d=100
    uint16 KEY_e=101
    uint16 KEY_f=102
    uint16 KEY_g=103
    uint16 KEY_h=104
    uint16 KEY_i=105
    uint16 KEY_j=106
    uint16 KEY_k=107
    uint16 KEY_l=108
    uint16 KEY_m=109
    uint16 KEY_n=110
    uint16 KEY_o=111
    uint16 KEY_p=112
    uint16 KEY_q=113
    uint16 KEY_r=114
    uint16 KEY_s=115
    uint16 KEY_t=116
    uint16 KEY_u=117
    uint16 KEY_v=118
    uint16 KEY_w=119
    uint16 KEY_x=120
    uint16 KEY_y=121
    uint16 KEY_z=122
    uint16 KEY_DELETE=127
    uint16 KEY_WORLD_0=160
    uint16 KEY_WORLD_1=161
    uint16 KEY_WORLD_2=162
    uint16 KEY_WORLD_3=163
    uint16 KEY_WORLD_4=164
    uint16 KEY_WORLD_5=165
    uint16 KEY_WORLD_6=166
    uint16 KEY_WORLD_7=167
    uint16 KEY_WORLD_8=168
    uint16 KEY_WORLD_9=169
    uint16 KEY_WORLD_10=170
    uint16 KEY_WORLD_11=171
    uint16 KEY_WORLD_12=172
    uint16 KEY_WORLD_13=173
    uint16 KEY_WORLD_14=174
    uint16 KEY_WORLD_15=175
    uint16 KEY_WORLD_16=176
    uint16 KEY_WORLD_17=177
    uint16 KEY_WORLD_18=178
    uint16 KEY_WORLD_19=179
    uint16 KEY_WORLD_20=180
    uint16 KEY_WORLD_21=181
    uint16 KEY_WORLD_22=182
    uint16 KEY_WORLD_23=183
    uint16 KEY_WORLD_24=184
    uint16 KEY_WORLD_25=185
    uint16 KEY_WORLD_26=186
    uint16 KEY_WORLD_27=187
    uint16 KEY_WORLD_28=188
    uint16 KEY_WORLD_29=189
    uint16 KEY_WORLD_30=190
    uint16 KEY_WORLD_31=191
    uint16 KEY_WORLD_32=192
    uint16 KEY_WORLD_33=193
    uint16 KEY_WORLD_34=194
    uint16 KEY_WORLD_35=195
    uint16 KEY_WORLD_36=196
    uint16 KEY_WORLD_37=197
    uint16 KEY_WORLD_38=198
    uint16 KEY_WORLD_39=199
    uint16 KEY_WORLD_40=200
    uint16 KEY_WORLD_41=201
    uint16 KEY_WORLD_42=202
    uint16 KEY_WORLD_43=203
    uint16 KEY_WORLD_44=204
    uint16 KEY_WORLD_45=205
    uint16 KEY_WORLD_46=206
    uint16 KEY_WORLD_47=207
    uint16 KEY_WORLD_48=208
    uint16 KEY_WORLD_49=209
    uint16 KEY_WORLD_50=210
    uint16 KEY_WORLD_51=211
    uint16 KEY_WORLD_52=212
    uint16 KEY_WORLD_53=213
    uint16 KEY_WORLD_54=214
    uint16 KEY_WORLD_55=215
    uint16 KEY_WORLD_56=216
    uint16 KEY_WORLD_57=217
    uint16 KEY_WORLD_58=218
    uint16 KEY_WORLD_59=219
    uint16 KEY_WORLD_60=220
    uint16 KEY_WORLD_61=221
    uint16 KEY_WORLD_62=222
    uint16 KEY_WORLD_63=223
    uint16 KEY_WORLD_64=224
    uint16 KEY_WORLD_65=225
    uint16 KEY_WORLD_66=226
    uint16 KEY_WORLD_67=227
    uint16 KEY_WORLD_68=228
    uint16 KEY_WORLD_69=229
    uint16 KEY_WORLD_70=230
    uint16 KEY_WORLD_71=231
    uint16 KEY_WORLD_72=232
    uint16 KEY_WORLD_73=233
    uint16 KEY_WORLD_74=234
    uint16 KEY_WORLD_75=235
    uint16 KEY_WORLD_76=236
    uint16 KEY_WORLD_77=237
    uint16 KEY_WORLD_78=238
    uint16 KEY_WORLD_79=239
    uint16 KEY_WORLD_80=240
    uint16 KEY_WORLD_81=241
    uint16 KEY_WORLD_82=242
    uint16 KEY_WORLD_83=243
    uint16 KEY_WORLD_84=244
    uint16 KEY_WORLD_85=245
    uint16 KEY_WORLD_86=246
    uint16 KEY_WORLD_87=247
    uint16 KEY_WORLD_88=248
    uint16 KEY_WORLD_89=249
    uint16 KEY_WORLD_90=250
    uint16 KEY_WORLD_91=251
    uint16 KEY_WORLD_92=252
    uint16 KEY_WORLD_93=253
    uint16 KEY_WORLD_94=254
    uint16 KEY_WORLD_95=255
    uint16 KEY_KP0=256
    uint16 KEY_KP1=257
    uint16 KEY_KP2=258
    uint16 KEY_KP3=259
    uint16 KEY_KP4=260
    uint16 KEY_KP5=261
    uint16 KEY_KP6=262
    uint16 KEY_KP7=263
    uint16 KEY_KP8=264
    uint16 KEY_KP9=265
    uint16 KEY_KP_PERIOD=266
    uint16 KEY_KP_DIVIDE=267
    uint16 KEY_KP_MULTIPLY=268
    uint16 KEY_KP_MINUS=269
    uint16 KEY_KP_PLUS=270
    uint16 KEY_KP_ENTER=271
    uint16 KEY_KP_EQUALS=272
    uint16 KEY_UP=273
    uint16 KEY_DOWN=274
    uint16 KEY_RIGHT=275
    uint16 KEY_LEFT=276
    uint16 KEY_INSERT=277
    uint16 KEY_HOME=278
    uint16 KEY_END=279
    uint16 KEY_PAGEUP=280
    uint16 KEY_PAGEDOWN=281
    uint16 KEY_F1=282
    uint16 KEY_F2=283
    uint16 KEY_F3=284
    uint16 KEY_F4=285
    uint16 KEY_F5=286
    uint16 KEY_F6=287
    uint16 KEY_F7=288
    uint16 KEY_F8=289
    uint16 KEY_F9=290
    uint16 KEY_F10=291
    uint16 KEY_F11=292
    uint16 KEY_F12=293
    uint16 KEY_F13=294
    uint16 KEY_F14=295
    uint16 KEY_F15=296
    uint16 KEY_NUMLOCK=300
    uint16 KEY_CAPSLOCK=301
    uint16 KEY_SCROLLOCK=302
    uint16 KEY_RSHIFT=303
    uint16 KEY_LSHIFT=304
    uint16 KEY_RCTRL=305
    uint16 KEY_LCTRL=306
    uint16 KEY_RALT=307
    uint16 KEY_LALT=308
    uint16 KEY_RMETA=309
    uint16 KEY_LMETA=310
    uint16 KEY_LSUPER=311
    uint16 KEY_RSUPER=312
    uint16 KEY_MODE=313
    uint16 KEY_COMPOSE=314
    uint16 KEY_HELP=315
    uint16 KEY_PRINT=316
    uint16 KEY_SYSREQ=317
    uint16 KEY_BREAK=318
    uint16 KEY_MENU=319
    uint16 KEY_POWER=320
    uint16 KEY_EURO=321
    uint16 KEY_UNDO=322
    uint16 MODIFIER_NONE=0
    uint16 MODIFIER_LSHIFT=1
    uint16 MODIFIER_RSHIFT=2
    uint16 MODIFIER_LCTRL=64
    uint16 MODIFIER_RCTRL=128
    uint16 MODIFIER_LALT=256
    uint16 MODIFIER_RALT=512
    uint16 MODIFIER_LMETA=1024
    uint16 MODIFIER_RMETA=2048
    uint16 MODIFIER_NUM=4096
    uint16 MODIFIER_CAPS=8192
    uint16 MODIFIER_MODE=16384
    uint16 MODIFIER_RESERVED=32768
    
    Header header
    uint16 code
    uint16 modifiers
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Key(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.code !== undefined) {
      resolved.code = msg.code;
    }
    else {
      resolved.code = 0
    }

    if (msg.modifiers !== undefined) {
      resolved.modifiers = msg.modifiers;
    }
    else {
      resolved.modifiers = 0
    }

    return resolved;
    }
};

// Constants for message
Key.Constants = {
  KEY_UNKNOWN: 0,
  KEY_FIRST: 0,
  KEY_BACKSPACE: 8,
  KEY_TAB: 9,
  KEY_CLEAR: 12,
  KEY_RETURN: 13,
  KEY_PAUSE: 19,
  KEY_ESCAPE: 27,
  KEY_SPACE: 32,
  KEY_EXCLAIM: 33,
  KEY_QUOTEDBL: 34,
  KEY_HASH: 35,
  KEY_DOLLAR: 36,
  KEY_AMPERSAND: 38,
  KEY_QUOTE: 39,
  KEY_LEFTPAREN: 40,
  KEY_RIGHTPAREN: 41,
  KEY_ASTERISK: 42,
  KEY_PLUS: 43,
  KEY_COMMA: 44,
  KEY_MINUS: 45,
  KEY_PERIOD: 46,
  KEY_SLASH: 47,
  KEY_0: 48,
  KEY_1: 49,
  KEY_2: 50,
  KEY_3: 51,
  KEY_4: 52,
  KEY_5: 53,
  KEY_6: 54,
  KEY_7: 55,
  KEY_8: 56,
  KEY_9: 57,
  KEY_COLON: 58,
  KEY_SEMICOLON: 59,
  KEY_LESS: 60,
  KEY_EQUALS: 61,
  KEY_GREATER: 62,
  KEY_QUESTION: 63,
  KEY_AT: 64,
  KEY_LEFTBRACKET: 91,
  KEY_BACKSLASH: 92,
  KEY_RIGHTBRACKET: 93,
  KEY_CARET: 94,
  KEY_UNDERSCORE: 95,
  KEY_BACKQUOTE: 96,
  KEY_A: 97,
  KEY_B: 98,
  KEY_C: 99,
  KEY_D: 100,
  KEY_E: 101,
  KEY_F: 102,
  KEY_G: 103,
  KEY_H: 104,
  KEY_I: 105,
  KEY_J: 106,
  KEY_K: 107,
  KEY_L: 108,
  KEY_M: 109,
  KEY_N: 110,
  KEY_O: 111,
  KEY_P: 112,
  KEY_Q: 113,
  KEY_R: 114,
  KEY_S: 115,
  KEY_T: 116,
  KEY_U: 117,
  KEY_V: 118,
  KEY_W: 119,
  KEY_X: 120,
  KEY_Y: 121,
  KEY_Z: 122,
  KEY_DELETE: 127,
  KEY_WORLD_0: 160,
  KEY_WORLD_1: 161,
  KEY_WORLD_2: 162,
  KEY_WORLD_3: 163,
  KEY_WORLD_4: 164,
  KEY_WORLD_5: 165,
  KEY_WORLD_6: 166,
  KEY_WORLD_7: 167,
  KEY_WORLD_8: 168,
  KEY_WORLD_9: 169,
  KEY_WORLD_10: 170,
  KEY_WORLD_11: 171,
  KEY_WORLD_12: 172,
  KEY_WORLD_13: 173,
  KEY_WORLD_14: 174,
  KEY_WORLD_15: 175,
  KEY_WORLD_16: 176,
  KEY_WORLD_17: 177,
  KEY_WORLD_18: 178,
  KEY_WORLD_19: 179,
  KEY_WORLD_20: 180,
  KEY_WORLD_21: 181,
  KEY_WORLD_22: 182,
  KEY_WORLD_23: 183,
  KEY_WORLD_24: 184,
  KEY_WORLD_25: 185,
  KEY_WORLD_26: 186,
  KEY_WORLD_27: 187,
  KEY_WORLD_28: 188,
  KEY_WORLD_29: 189,
  KEY_WORLD_30: 190,
  KEY_WORLD_31: 191,
  KEY_WORLD_32: 192,
  KEY_WORLD_33: 193,
  KEY_WORLD_34: 194,
  KEY_WORLD_35: 195,
  KEY_WORLD_36: 196,
  KEY_WORLD_37: 197,
  KEY_WORLD_38: 198,
  KEY_WORLD_39: 199,
  KEY_WORLD_40: 200,
  KEY_WORLD_41: 201,
  KEY_WORLD_42: 202,
  KEY_WORLD_43: 203,
  KEY_WORLD_44: 204,
  KEY_WORLD_45: 205,
  KEY_WORLD_46: 206,
  KEY_WORLD_47: 207,
  KEY_WORLD_48: 208,
  KEY_WORLD_49: 209,
  KEY_WORLD_50: 210,
  KEY_WORLD_51: 211,
  KEY_WORLD_52: 212,
  KEY_WORLD_53: 213,
  KEY_WORLD_54: 214,
  KEY_WORLD_55: 215,
  KEY_WORLD_56: 216,
  KEY_WORLD_57: 217,
  KEY_WORLD_58: 218,
  KEY_WORLD_59: 219,
  KEY_WORLD_60: 220,
  KEY_WORLD_61: 221,
  KEY_WORLD_62: 222,
  KEY_WORLD_63: 223,
  KEY_WORLD_64: 224,
  KEY_WORLD_65: 225,
  KEY_WORLD_66: 226,
  KEY_WORLD_67: 227,
  KEY_WORLD_68: 228,
  KEY_WORLD_69: 229,
  KEY_WORLD_70: 230,
  KEY_WORLD_71: 231,
  KEY_WORLD_72: 232,
  KEY_WORLD_73: 233,
  KEY_WORLD_74: 234,
  KEY_WORLD_75: 235,
  KEY_WORLD_76: 236,
  KEY_WORLD_77: 237,
  KEY_WORLD_78: 238,
  KEY_WORLD_79: 239,
  KEY_WORLD_80: 240,
  KEY_WORLD_81: 241,
  KEY_WORLD_82: 242,
  KEY_WORLD_83: 243,
  KEY_WORLD_84: 244,
  KEY_WORLD_85: 245,
  KEY_WORLD_86: 246,
  KEY_WORLD_87: 247,
  KEY_WORLD_88: 248,
  KEY_WORLD_89: 249,
  KEY_WORLD_90: 250,
  KEY_WORLD_91: 251,
  KEY_WORLD_92: 252,
  KEY_WORLD_93: 253,
  KEY_WORLD_94: 254,
  KEY_WORLD_95: 255,
  KEY_KP0: 256,
  KEY_KP1: 257,
  KEY_KP2: 258,
  KEY_KP3: 259,
  KEY_KP4: 260,
  KEY_KP5: 261,
  KEY_KP6: 262,
  KEY_KP7: 263,
  KEY_KP8: 264,
  KEY_KP9: 265,
  KEY_KP_PERIOD: 266,
  KEY_KP_DIVIDE: 267,
  KEY_KP_MULTIPLY: 268,
  KEY_KP_MINUS: 269,
  KEY_KP_PLUS: 270,
  KEY_KP_ENTER: 271,
  KEY_KP_EQUALS: 272,
  KEY_UP: 273,
  KEY_DOWN: 274,
  KEY_RIGHT: 275,
  KEY_LEFT: 276,
  KEY_INSERT: 277,
  KEY_HOME: 278,
  KEY_END: 279,
  KEY_PAGEUP: 280,
  KEY_PAGEDOWN: 281,
  KEY_F1: 282,
  KEY_F2: 283,
  KEY_F3: 284,
  KEY_F4: 285,
  KEY_F5: 286,
  KEY_F6: 287,
  KEY_F7: 288,
  KEY_F8: 289,
  KEY_F9: 290,
  KEY_F10: 291,
  KEY_F11: 292,
  KEY_F12: 293,
  KEY_F13: 294,
  KEY_F14: 295,
  KEY_F15: 296,
  KEY_NUMLOCK: 300,
  KEY_CAPSLOCK: 301,
  KEY_SCROLLOCK: 302,
  KEY_RSHIFT: 303,
  KEY_LSHIFT: 304,
  KEY_RCTRL: 305,
  KEY_LCTRL: 306,
  KEY_RALT: 307,
  KEY_LALT: 308,
  KEY_RMETA: 309,
  KEY_LMETA: 310,
  KEY_LSUPER: 311,
  KEY_RSUPER: 312,
  KEY_MODE: 313,
  KEY_COMPOSE: 314,
  KEY_HELP: 315,
  KEY_PRINT: 316,
  KEY_SYSREQ: 317,
  KEY_BREAK: 318,
  KEY_MENU: 319,
  KEY_POWER: 320,
  KEY_EURO: 321,
  KEY_UNDO: 322,
  MODIFIER_NONE: 0,
  MODIFIER_LSHIFT: 1,
  MODIFIER_RSHIFT: 2,
  MODIFIER_LCTRL: 64,
  MODIFIER_RCTRL: 128,
  MODIFIER_LALT: 256,
  MODIFIER_RALT: 512,
  MODIFIER_LMETA: 1024,
  MODIFIER_RMETA: 2048,
  MODIFIER_NUM: 4096,
  MODIFIER_CAPS: 8192,
  MODIFIER_MODE: 16384,
  MODIFIER_RESERVED: 32768,
}

module.exports = Key;
