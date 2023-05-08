// Decode uplink function for chirpstack.
//
// Input is an object with the following fields:
// - bytes = Byte array containing the uplink payload, e.g. [255, 230, 255, 0]
// - fPort = Uplink fPort.
// - variables = Object containing the configured device variables.
//
// Output must be an object with the following fields:
// - data = Object representing the decoded payload.
function decodeUplink(input) {
    return {
        data: {
            lat: bytesToDouble(input.bytes.splice(0,8)),
            long: bytesToDouble(input.bytes.splice(0,8))
        }
    };
}

function bytesToDouble(byteArray) {
    let data = new DataView(new Uint8Array(byteArray).buffer);
    return data.getFloat64(0,true /* little endian */);
}