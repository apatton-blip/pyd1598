from enum import IntEnum
import difflib

class OperationMode(IntEnum):
    FORCED_READOUT = 0
    INTERRUPT_READOUT = 1
    WAKEUP = 2


class SignalSource(IntEnum):
    BPF = 0
    LPF = 1
    TEMPERATURE = 3

CONFIG_RAW_MASK = 0x1FFFFFF
RESERVED_BIT_MASK = 0x1A
EMPTY_CONFIG = 0x10

def decode_config(config: int):
    """
    Replace the bit positions below with the actual values from the datasheet.
    """

    if (config > CONFIG_RAW_MASK):
        raise Exception("Invalid Config: Configuration must be a value [0x0, 0x1FFFFFF]\n")
    if ((config & RESERVED_BIT_MASK) ^ EMPTY_CONFIG):
        raise Exception("Invalid Config: Reserved are not set.\n")

    # EXAMPLE bit mappings — these are placeholders!
    threshold = (config >> 17) & 0xFF
    blind_time = (config >> 13) & 0xF
    pulse_counter = (config >> 11) & 0x3
    window_time = (config >> 9) & 0x3
    op_mode = (config >> 7) & 0x3
    sig_src = (config >> 5) & 0x3
    hpf_cutoff = (config >> 2) & 0x1
    count_mode = (config >> 0) & 0x1

    return {
        "threshold": threshold,
        "blind_time": blind_time,
        "pulse_counter": pulse_counter,
        "window_time": window_time,
        "op_mode": op_mode,
        "sig_src": sig_src,
        "hpf_cutoff": hpf_cutoff,
        "count_mode": count_mode,
    }

def make_config_readable(config_hex: int):
    cfg = decode_config(config_hex)

    threshold = cfg["threshold"]
    blind_time = cfg["blind_time"]
    pulse_counter = cfg["pulse_counter"]
    window_time = cfg["window_time"]

    op_mode = cfg["op_mode"]
    sig_src = cfg["sig_src"]

    threshold_uv = threshold * 6.5

    blind_time_seconds = 0.5 + 0.5 * blind_time

    op_mode_text = {
        OperationMode.FORCED_READOUT: "FORCED READOUT",
        OperationMode.INTERRUPT_READOUT: "INTERRUPT READOUT",
    }.get(op_mode, "WAKEUP")

    sig_src_text = {
        SignalSource.BPF: "BPF",
        SignalSource.LPF: "LPF",
    }.get(sig_src, "TEMPERATURE")

    config_readable = ""
    config_readable += (f"\n------------ CONFIG: 0x{config_hex:07X} -------------\n")
    config_readable += (f"Threshold       : {threshold_uv:.1f} uV\n")
    config_readable += (f"Blind Time      : {blind_time_seconds:.1f}s\n")
    config_readable += (f"Pulse Counter   : {pulse_counter + 1} PULSE\n{'S' if pulse_counter else ''}")
    config_readable += (f"Window Time     : {2 + 2 * window_time}s\n")
    config_readable += (f"Operation Modes : {op_mode_text}\n")
    config_readable += (f"Signal Source   : {sig_src_text}\n")
    config_readable += (f"HPF Cutoff      : {'0.2' if cfg['hpf_cutoff'] else '0.4'} Hz\n")
    config_readable += (f"Count Mode      : {'WITHOUT' if cfg['count_mode'] else 'WITH'} SIGN CHANGE\n")
    config_readable += ("--------------------------------------------\n")
    return config_readable

def color_diff(old_str, new_str):
    """
    Highlights differences between two strings using standard ANSI escape codes.
    Deletions are Red with a Strikethrough. Additions are Green.
    """
    # ANSI Escape Sequences
    RESET = "\033[0m"
    RED_STRIKE = "\033[31;9m"  # 31 = Red, 9 = Strikethrough
    GREEN = "\033[32m"         # 32 = Green
    if (old_str == new_str):
        return "Strings are Identical!"
    matcher = difflib.SequenceMatcher(None, old_str, new_str)
    result = []
    
    for tag, o_start, o_end, n_start, n_end in matcher.get_opcodes():
        if tag == 'equal':
            result.append(old_str[o_start:o_end])
        elif tag == 'delete':
            # Text removed from the original string
            result.append(f"{RED_STRIKE}{old_str[o_start:o_end]}{RESET}")
        elif tag == 'insert':
            # Text added to the new string
            result.append(f"{GREEN}{new_str[n_start:n_end]}{RESET}")
        elif tag == 'replace':
            # Text substituted (strike out old, add green new)
            result.append(f"{RED_STRIKE}{old_str[o_start:o_end]}{RESET}")
            result.append(f"{GREEN}{new_str[n_start:n_end]}{RESET}")
            
    return "".join(result)

if __name__ == '__main__':
    # bin1 = int('0000000000000000000010000', 2)
    bin2 = int('0000000000000000100010000', 2)
    # bin3 = int('0010110100000000100010000', 2)

    # bin1 = int('0000000000000000000110000', 2)
    # bin2 = int('0000000000000000000110000', 2)

    # # bins = [bin1, bin2]
    # bins = [0x2A4211, 0x0B79840, 0x0B79844]

    # for index, bin in enumerate(bins):
    #     print(make_config_readable(bin))
    #     if not bin is bins[-1]:
    #         print(color_diff(\
    #             make_config_readable(bin),\
    #             make_config_readable(bins[index + 1])\
    #         ))

    hexa = 0x0000004
    print(make_config_readable(hexa))