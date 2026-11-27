import zlib, re, sys

path = r"D:\硕士生涯\FOC-learning\myFOC\drv8313.pdf"
data = open(path, "rb").read()
print("file size:", len(data))

# Find all stream objects and try to decompress
texts = []
# Simple approach: find 'stream' ... 'endstream' blocks, try flate decode
for m in re.finditer(rb'stream\r?\n(.*?)endstream', data, re.DOTALL):
    raw = m.group(1)
    raw = raw.strip(b'\r\n')
    for attempt in (raw, raw + b'\n'):
        try:
            dec = zlib.decompress(attempt)
            texts.append(dec)
            break
        except Exception:
            try:
                dec = zlib.decompress(attempt, -15)
                texts.append(dec)
                break
            except Exception:
                pass

print("decompressed streams:", len(texts))
alltext = b"\n".join(texts)
# Extract text between BT/ET (Tj/TJ operators)
found = re.findall(rb'\((.*?)\)\s*Tj', alltext, re.DOTALL)
print("Tj strings:", len(found))
out = []
for f in found:
    s = f.replace(b'\\(', b'(').replace(b'\\)', b')').replace(b'\\\\', b'\\')
    out.append(s.decode('latin1', 'ignore'))
joined = "\n".join(out)
# Write to file
with open(r"D:\硕士生涯\FOC-learning\myFOC\drv8313_text.txt", "w", encoding="utf-8") as fh:
    fh.write(joined)
print("wrote drv8313_text.txt, chars:", len(joined))
