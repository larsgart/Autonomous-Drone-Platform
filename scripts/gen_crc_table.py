def generate_crc_table(poly: int, width: int = 16, reflect: bool = False):
    """Generate a CRC lookup table for given polynomial.

    Args:
        poly (int): The polynomial (e.g., 0x1021 for CRC-16-CCITT).
        width (int): Width of CRC in bits (8, 16, 32).
        reflect (bool): If True, process bits LSB-first (reflected CRCs).

    Returns:
        list[int]: 256-entry CRC lookup table.
    """
    topbit = 1 << (width - 1)
    mask = (1 << width) - 1
    table = []

    for byte in range(256):
        if reflect:
            remainder = byte
        else:
            remainder = byte << (width - 8)

        for _ in range(8):
            if reflect:
                if remainder & 0x0001:
                    remainder = (remainder >> 1) ^ poly
                else:
                    remainder >>= 1
            else:
                if remainder & topbit:
                    remainder = ((remainder << 1) ^ poly) & mask
                else:
                    remainder = (remainder << 1) & mask

        table.append(remainder & mask)

    return table


if __name__ == "__main__":
    # Example: CRC-16-CCITT-FALSE
    POLY = 0x1021
    WIDTH = 16
    REFLECT = False

    table = generate_crc_table(POLY, WIDTH, REFLECT)

    # Print as C array
    print("static const uint16_t crc16_table[256] = {")
    for i in range(0, 256, 8):
        row = ", ".join(f"0x{val:04X}" for val in table[i:i+8])
        print("    " + row + ",")
    print("};")
