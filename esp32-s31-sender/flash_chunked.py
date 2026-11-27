#!/usr/bin/env python3
"""把过大的 app 固件拆成若干块，逐块写进 Flash。

为什么需要它
------------
这颗 ESP32-S31（ROM 加载器 + 当前 esptool 5.x）在写 3.7MB 这种大固件时，
不论波特率 460800 还是 230400、不论走 ROM 还是 stub，都会在传大数据块时
失去响应（`No more data to read` / `Failed to start stub flasher`）。
而 bootloader、分区表这种几十 KB 的小块从来没失败过。

所以这里把 `sample_project.bin` 按字节切成若干段，第 i 段写到
`base + i*chunk` 的位置。因为 app.bin 就是一段连续的 Flash 镜像，
分块写入和一次写完在 Flash 上的结果完全一致。

⚠ 块大小必须是 **4096 的整数倍**（Flash 一个扇区）。
  esptool 写入前会把涉及的扇区整片擦掉，如果切点落在扇区中间，
  擦这一块的时候会把上一块的尾巴一起抹掉，镜像就坏了 ——
  症状是烧完板子无限重启（rst:0x3 SW_SYS_RESET）。
  默认用 0x100000（1MiB）就是为了保证这点。

用法
----
    python flash_chunked.py --port COM10 --bin build/sample_project.bin

只做校验不写：
    python flash_chunked.py --port COM10 --bin build/sample_project.bin --verify-only
"""
import argparse
import hashlib
import os
import subprocess
import sys
import tempfile
import time


def run_esptool(args):
    """跑一次 esptool，返回 (成功?, 输出文本)"""
    cmd = [sys.executable, "-m", "esptool"] + args
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")
    out = (p.stdout or "") + (p.stderr or "")
    return p.returncode == 0, out.strip()


def port_openable(port):
    """确认串口没被占用（上一次失败常把 CP210x 卡住）"""
    try:
        import serial  # pyserial，esptool 自带
    except ImportError:
        return True, "无法 import serial，跳过检查"
    try:
        s = serial.Serial(port, 115200, timeout=0.2)
        s.close()
        return True, "ok"
    except Exception as exc:
        return False, str(exc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", default="460800")
    ap.add_argument("--bin", default="build/sample_project.bin")
    ap.add_argument("--base", default="0x10000")
    ap.add_argument("--chunk", type=int, default=0x100000,
                    help="每块多少字节，必须是 4096 的整数倍（默认 0x100000 = 1MiB）")
    ap.add_argument("--chip", default="esp32s31")
    ap.add_argument("--verify-only", action="store_true",
                    help="不写，只把 Flash 里的 app 读回来比对 MD5")
    ap.add_argument("--readback", action="store_true",
                    help="写完后额外把整段读回来比对 MD5（慢，且可能因大传输中断）")
    ap.add_argument("--retries", type=int, default=3, help="每块失败重试次数")
    args = ap.parse_args()

    if not os.path.exists(args.bin):
        sys.exit("找不到固件: " + args.bin)

    base = int(args.base, 0)
    if base % 4096 or args.chunk % 4096:
        sys.exit("块大小和起始地址都必须是 4096 的整数倍"
                 "（base=0x{:X}, chunk={}），否则擦扇区会抹掉相邻块的尾部"
                 .format(base, args.chunk))
    data = open(args.bin, "rb").read()
    total_md5 = hashlib.md5(data).hexdigest()
    parts = [data[i:i + args.chunk] for i in range(0, len(data), args.chunk)]

    print("固件   : %s" % os.path.abspath(args.bin))
    print("大小   : {:,} 字节 (0x{:X})".format(len(data), len(data)))
    print("MD5    : %s" % total_md5)
    print("分块   : {} 块，每块 ≤ {:,} 字节".format(len(parts), args.chunk))
    for i, p in enumerate(parts):
        print("         #{} 0x{:06X}  {:>9,} 字节".format(i, base + i * args.chunk, len(p)))
    print()

    ok, msg = port_openable(args.port)
    if not ok:
        sys.exit("串口 %s 打不开：%s\n→ 把 USB 线拔掉等 3 秒再插回来" % (args.port, msg))

    tmpdir = tempfile.mkdtemp(prefix="fwchunk_")
    if not args.verify_only:
        for i, part in enumerate(parts):
            path = os.path.join(tmpdir, "chunk%02d.bin" % i)
            with open(path, "wb") as f:
                f.write(part)
            offset = base + i * args.chunk

            for attempt in range(1, args.retries + 1):
                print("写 #{}/{}  0x{:06X}  {:,} 字节  第 {} 次尝试 …".format(
                    i, len(parts), offset, len(part), attempt), flush=True)
                t0 = time.time()
                success, out = run_esptool([
                    "--chip", args.chip, "-p", args.port, "-b", args.baud,
                    "--before", "default-reset", "--after", "no-reset",
                    "--connect-attempts", "5",
                    "write-flash", "--flash-mode", "dio",
                    "--flash-size", "16MB", "--flash-freq", "80m",
                    hex(offset), path,
                ])
                tail = [l for l in out.splitlines()
                        if any(k in l for k in ("Wrote", "Hash", "ERROR", "fatal", "Compressed"))]
                for l in tail[-3:]:
                    print("        " + l)
                if success:
                    print("        [OK] {:.1f} 秒".format(time.time() - t0))
                    break
                print("        [!!] 失败，等待 3 秒重试")
                time.sleep(3)
            else:
                sys.exit("块 #%d 连续 %d 次失败，停止。先拔插 USB 再重跑。" % (i, args.retries))

    # ---- 读回校验（可选）----
    # 每写一块，esptool 自己都会让 stub 校验一次 hash（日志里的 "Hash of data verified"），
    # 这已经能保证写进去的字节是对的。整段读回只是再保险一次，
    # 但读 3.7MB 同样是一次大传输，也可能断，所以默认不做，需要时加 --readback。
    if not (args.readback or args.verify_only):
        print("\n[OK] 全部 {} 块写入完成，每块 hash 已校验".format(len(parts)))
        return

    print("\n读回校验整段 app（{:,} 字节）…".format(len(data)))
    readback = os.path.join(tmpdir, "readback.bin")
    success, out = run_esptool([
        "--chip", args.chip, "-p", args.port, "-b", args.baud,
        "--before", "default-reset", "--after", "hard-reset",
        "--connect-attempts", "5",
        "read-flash", hex(base), str(len(data)), readback,
    ])
    if not success:
        sys.exit("读回失败：\n" + out[-800:])
    got = open(readback, "rb").read()
    got_md5 = hashlib.md5(got).hexdigest()
    print("写入 MD5: %s" % total_md5)
    print("读回 MD5: %s" % got_md5)
    if got_md5 == total_md5:
        print("\n[OK] Flash 内容与固件逐字节一致")
    else:
        sys.exit("\n[!!] 读回的 MD5 不一致，Flash 里是错的")

    try:
        os.remove(readback)
    except OSError:
        pass


if __name__ == "__main__":
    main()
