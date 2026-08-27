# -*- coding: utf-8 -*-
import argparse, json
from services.pallet_hole_recognition_service import PalletHoleRecognitionService

p = argparse.ArgumentParser()
p.add_argument('--rgb', default='examples/pallet_rgbd/pallet1.jpg')
p.add_argument('--depth', default='examples/pallet_rgbd/pallet1_depth.png')
args = p.parse_args()
result = PalletHoleRecognitionService().recognize(args.rgb, args.depth)
print(json.dumps(result, ensure_ascii=False, indent=2))
