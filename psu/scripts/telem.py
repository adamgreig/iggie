import struct
import serial

MAGIC = 0x74656c65


def main():
    s = serial.Serial("/dev/ttyACM1", 1000000)

    while True:
        # Align to magic
        rx = s.read(4)
        while struct.unpack("<I", rx)[0] != MAGIC:
            s.read()
            rx = s.read(4)
        s.read(7*4)

        # Read telemetry
        blink = "."
        while True:
            rx = s.read(8*4)
            (magic, v_in, i_in, v_out, i_out,
             v_q, i_q, ref_v_q, ref_i_q) = struct.unpack("<IffffffHH", rx)
            if magic != MAGIC:
                break
            print(f"{blink} V_in: {v_in: 3.02f}V    I_in: {i_in: 3.02f}A    "
                  f"V_out: {v_out: 3.00f}V    I_out: {1000*i_out: 3.01f}mA    "
                  f"V_Q: {v_q: 3.02f}V    I_Q: { i_q: 3.02f}A    "
                  f"Ref V_Q: {ref_v_q:05}    Ref I_Q: {ref_i_q:05}",
                  end="\r", flush=True)
            blink = " " if blink == "." else "."


if __name__ == "__main__":
    main()
