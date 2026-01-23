
data = [0xe5b7, 0x8788, 0x8de6, 0xe5a2, 0xb088, 0x97e6, 0xe5a0, 0xa5b7, 0x85e5, 0xe6b7, 0x8b89, 0x8ae7, 0xe6b6, 0x8180]

# 方法 1: 大端字节序
bytes_be = b''
for d in data:
    bytes_be += d.to_bytes(2, byteorder='big')

# 方法 2: 小端字节序
bytes_le = b''
for d in data:
    bytes_le += d.to_bytes(2, byteorder='little')

encodings = ['utf-8', 'gbk', 'utf-16', 'utf-16be', 'utf-16le']

print("--- Big Endian Bytes ---")
print(bytes_be.hex())
for enc in encodings:
    try:
        print(f"{enc}: {bytes_be.decode(enc)}")
    except:
        pass # print(f"{enc}: failed")

print("\n--- Little Endian Bytes (Shifted) ---")
try:
    print(f"utf-8 (skip 1 byte): {bytes_le[1:].decode('utf-8')}")
except Exception as e:
    print(f"utf-8 (skip 1 byte): failed {e}")

try:
    print(f"utf-8 (full): {bytes_le.decode('utf-8')}")
except Exception as e:
    print(f"utf-8 (full): failed {e}")

