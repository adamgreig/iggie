import struct
import serial

MAGIC = 0x74656c65

FAULTS = {
    0: "None      ",
    1: "No RUN    ",
    2: "Vout Limit",
    3: "Iout Limit",
    4: "No I_Q    ",
    5: "No Vout   ",
    6: "VIn Low   ",
    7: "VIn High  ",
    8: "IIn High  ",
}


STATES = {
    0: "Stopped",
    1: "Running",
    2: "Fault  ",
}


def main():
    s = serial.Serial("/dev/ttyACM1", 1000000)

    while True:
        # Align to magic
        rx = s.read(4)
        while struct.unpack("<I", rx)[0] != MAGIC:
            s.read()
            rx = s.read(4)
        s.read(8*4)

        # Read telemetry
        blink = "."
        while True:
            rx = s.read(9*4)
            (magic, v_in, i_in, v_out, i_out,
             v_q, i_q, pid_i, ref_i_q, fault, state) = struct.unpack(
                "<IfffffffHBB", rx)
            if magic != MAGIC:
                break
            fault = FAULTS.get(fault, "?")
            state = STATES.get(state, "?")
            print(f"{blink} V_in: {v_in: 3.02f}V    I_in: {i_in: 3.02f}A    "
                  f"V_out: {v_out: 3.00f}V    I_out: {1000*i_out: 3.01f}mA    "
                  f"V_Q: {v_q: 3.02f}V    I_Q: { i_q: 3.02f}A    "
                  f"PID I: {pid_i:5.01f}    ",
                  f"Ref I_Q: {ref_i_q:05}    Fault: {fault}  State: {state}",
                  " "*10,
                  end="\r", flush=True)
            blink = " " if blink == "." else "."


if __name__ == "__main__":
    main()
