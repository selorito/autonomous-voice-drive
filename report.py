"""
results.csv -> sunum icin ozet metrikler.
Kullanim: python3 report.py [results.csv]
"""
import csv
import statistics
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else "results.csv"
rows = list(csv.DictReader(open(path, encoding="utf-8")))
if not rows:
    sys.exit("results.csv bos")

n = len(rows)
correct = sum(int(r["correct"]) for r in rows)
lat = sorted(float(r["asr_ms"]) for r in rows)
p95 = lat[min(len(lat) - 1, int(0.95 * len(lat)))]

cmd_rows = [r for r in rows if r["expected"] != "NONE"]
none_rows = [r for r in rows if r["expected"] == "NONE"]
stop_rows = [r for r in rows if r["expected"] == "STOP"]

# En kritik iki metrik:
#  - Yanlis hareket: arac hareket etmemeliyken bir hareket komutu uretilmesi
#  - Kacirilan STOP: "dur" denmesine ragmen STOP uretilmemesi
false_action = sum(r["predicted"] not in ("NONE", "STOP") for r in none_rows)
missed_stop = sum(r["predicted"] != "STOP" for r in stop_rows)
wrong_cmd = sum(r["predicted"] not in ("NONE", r["expected"]) for r in cmd_rows)

print(f"Toplam deneme      : {n}  ({len({r['speaker'] for r in rows})} kişi)")
print(f"Genel doğruluk     : %{100 * correct / n:.1f}  ({correct}/{n})")
if cmd_rows:
    ok = sum(int(r["correct"]) for r in cmd_rows)
    print(f"Komut doğruluğu    : %{100 * ok / len(cmd_rows):.1f}  ({ok}/{len(cmd_rows)})")
print(f"Yanlış komut       : {wrong_cmd}   (örn. LEFT yerine RIGHT)")
print(f"Yanlış hareket     : {false_action}/{len(none_rows)}   (hareket etmemeliyken etti)")
print(f"Kaçırılan STOP     : {missed_stop}/{len(stop_rows)}")
print(f"ASR gecikmesi      : ort {statistics.mean(lat):.0f} ms, medyan {statistics.median(lat):.0f} ms, p95 {p95:.0f} ms")


def group(key):
    g = defaultdict(lambda: [0, 0])
    for r in rows:
        g[r[key]][0] += int(r["correct"])
        g[r[key]][1] += 1
    return g


for key, title in [("expected", "Komut bazında"), ("speaker", "Kişi bazında"), ("noise", "Ortam bazında")]:
    print(f"\n{title}:")
    for k, (c, t) in sorted(group(key).items()):
        print(f"  {k:<10} %{100 * c / t:5.1f}  ({c}/{t})")

errors = [r for r in rows if r["correct"] == "0"]
if errors:
    print("\nHatalar:")
    for r in errors:
        print(f"  [{r['speaker']}/{r['noise']}] '{r['prompt']}' -> duyulan '{r['heard']}' "
              f"-> {r['predicted']} (beklenen {r['expected']})")
