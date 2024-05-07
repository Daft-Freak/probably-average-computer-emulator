#pragma once

// break codes are | 0x80
enum class XTScancode
{
    Invalid = 0x00,

    A = 0x1E,
    B = 0x30,
    C = 0x2E,
    D = 0x20,
    E = 0x12,
    F = 0x21,
    G = 0x22,
    H = 0x23,
    I = 0x17,
    J = 0x24,
    K = 0x25,
    L = 0x26,
    M = 0x32,
    N = 0x31,
    O = 0x18,
    P = 0x19,
    Q = 0x10,
    R = 0x13,
    S = 0x1F,
    T = 0x14,
    U = 0x16,
    V = 0x2F,
    W = 0x22,
    X = 0x2D,
    Y = 0x15,
    Z = 0x2C,
    
    _1 = 0x02,
    _2 = 0x03,
    _3 = 0x04,
    _4 = 0x05,
    _5 = 0x06,
    _6 = 0x07,
    _7 = 0x08,
    _8 = 0x09,
    _9 = 0x0A,
    _0 = 0x0B,

    Return = 0x1C,
    Escape = 0x01,
    Backspace = 0x0E,
    Tab = 0x0F,
    Space = 0x39,

    Minus = 0x0C,
    Equals = 0x0D,
    LeftBracket = 0x1A,
    RightBracket = 0x1B,
    Backslash = 0x2B,
    Semicolon = 0x27,
    Apostrophe = 0x28,
    Grave = 0x29,
    Comma = 0x33,
    Period = 0x34,
    Slash = 0x35,

    CapsLock = 0x3A,

    F1 = 0x3B,
    F2 = 0x3C,
    F3 = 0x3D,
    F4 = 0x3E,
    F5 = 0x3F,
    F6 = 0x40,
    F7 = 0x41,
    F8 = 0x42,
    F9 = 0x43,
    F10 = 0x44,
    F11 = 0x57,
    F12 = 0x58,

    // PrintScreen = 0xE037,
    ScrollLock = 0x46,
    // Pause = 0xE11D45 E19DC5, // immediate break?
    // Insert = 0xE052,
    
    // Home = 0xE047,
    // PageUp = 0xE049,
    // Delete = 0xE043,
    // End = 0xE04F,
    // PageDown = 0xE051,
    // Right = 0xE04D,
    // Left = 0xE04B,
    // Down = 0xE050,
    // Up = 0xE048,

    NumLock = 0x45,

    // KPDivide = 0xE035,
    KPMultiply = 0x37,
    KPMinus = 0x4A,
    KPPlus = 0x4E,
    //KPEnter = 0xE01C,
    KP1 = 0x4F,
    KP2 = 0x50,
    KP3 = 0x51,
    KP4 = 0x4B,
    KP5 = 0x4C,
    KP6 = 0x4D,
    KP7 = 0x47,
    KP8 = 0x48,
    KP9 = 0x49,
    KP0 = 0x52,
    KPPeriod = 0x53,

    NonUSBackslash = 0x56,

    // Application = 0xE05D,
    // Power = 0xE05E,

    KPEquals = 0x59,

    // F13-24 are 64-6E and 76

    // skipping unassigned...

    KPComma = 0x7E,

    International1 = 0x73,
    International2 = 0x70,
    International3 = 0x7D,
    International4 = 0x79,
    International5 = 0x7B,
    International6 = 0x5C,
    Lang1 = 0xF2,
    Lang2 = 0xF1,
    Lang3 = 0x78,
    Lang4 = 0x77,
    Lang5 = 0x76,

    LeftCtrl = 0x1D,
    LeftShift = 0x2A,
    LeftAlt = 0x38,
    // LeftGUI = 0xE05B,
    // RightCtrl = 0xE01D,
    RightShift = 0x36,
    // RightAlt = 0xE038,
    // RightGUI = 0xE05C,

    // some media keys with two byte codes...
};