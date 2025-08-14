#!/usr/bin/env python3
"""
slam_to_gps_icp.py  ―  SLAM XYZ ↔ GPS LLA  (Umeyama + multi‑round numpy ICP)
------------------------------------------------------------------------
New features: Print  ▸ Initial Umeyama RMSE
                    ▸ ICP RMSE per round
                    ▸ Final ICP RMSE
Support: Original ECEF implementation and pymap3d implementation (default uses pymap3d)
"""
import json, argparse
from pathlib import Path
import numpy as np
import pandas as pd
from math import sin, cos, radians
from pyproj import Transformer            # 4979↔4978
from pymap3d import geodetic2enu, enu2geodetic  # New: pymap3d support

# ---------- Original implementation (based on ECEF) ---------- #
# _lla2ecef = Transformer.from_crs("epsg:4979", "epsg:4978", always_xy=True)
# _ecef2lla = Transformer.from_crs("epsg:4978", "epsg:4979", always_xy=True)

# def lla_to_ecef(lon, lat, alt): return _lla2ecef.transform(lon, lat, alt)
# def ecef_to_lla(x, y, z): lon, lat, alt = _ecef2lla.transform(x, y, z); return np.c_[lat, lon, alt]

# def lla_to_enu(lon, lat, alt, lon0, lat0, alt0):
#     x, y, z   = lla_to_ecef(lon, lat, alt)
#     x0, y0, z0 = lla_to_ecef(lon0, lat0, alt0)
#     dx, dy, dz = x - x0, y - y0, z - z0
#     lon0_r, lat0_r = radians(lon0), radians(lat0)
#     sL, cL = sin(lon0_r), cos(lon0_r)
#     sB, cB = sin(lat0_r), cos(lat0_r)

#     E = -sL*dx + cL*dy
#     N = -sB*cL*dx - sB*sL*dy + cB*dz
#     U =  cB*cL*dx + cB*sL*dy + sB*dz
#     return np.c_[E, N, U]

# def enu_to_lla(E, N, U, lon0, lat0, alt0):
#     lon0_r, lat0_r = radians(lon0), radians(lat0)
#     sL, cL, sB, cB = sin(lon0_r), cos(lon0_r), sin(lat0_r), cos(lat0_r)
#     dx = -sL*E - sB*cL*N + cB*cL*U
#     dy =  cL*E - sB*sL*N + cB*sL*U
#     dz =             cB*N + sB*U
#     x0, y0, z0 = lla_to_ecef(lon0, lat0, alt0)
#     return ecef_to_lla(x0+dx, y0+dy, z0+dz)

# ---------- New implementation (based on pymap3d) ---------- #
def lla_to_enu(lon, lat, alt, lon0, lat0, alt0):
    e, n, u = geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
    return np.c_[e, n, u]

def enu_to_lla(E, N, U, lon0, lat0, alt0):
    lat, lon, alt = enu2geodetic(E, N, U, lat0, lon0, alt0)
    return np.c_[lat, lon, alt]

# ---------- Umeyama ---------- #
def umeyama(src, dst, with_scale=True):
    mu_s = src.mean(axis=1, keepdims=True)
    mu_d = dst.mean(axis=1, keepdims=True)
    src_c, dst_c = src - mu_s, dst - mu_d
    U,S,Vt = np.linalg.svd(dst_c @ src_c.T / src.shape[1])
    R = U @ Vt
    if np.linalg.det(R) < 0: Vt[-1]*=-1; R = U @ Vt
    s = 1.0
    if with_scale:
        s = S.sum() / (src_c**2).sum() * src.shape[1]
    t = mu_d - s*R@mu_s
    return s,R,t

# ---------- ICP ---------- #
try:
    from scipy.spatial import cKDTree as KDTree
    _USE_KD = True
except ImportError:
    _USE_KD = False

def nn(src, dst):
    if _USE_KD:
        idx = KDTree(dst).query(src)[1]
        diff = src - dst[idx]
        return idx, (diff**2).sum(-1)
    diff = src[:,None,:]-dst[None,:,:]
    dist2 = (diff**2).sum(-1)
    idx = dist2.argmin(1)
    return idx, dist2[np.arange(len(src)), idx]

def icp_sim3(src, dst, s0, R0, t0, max_iter=30, eps=1e-6):
    s,R,t = s0, R0, t0
    prev_err = None
    for it in range(max_iter):
        src_t = (s*(R@src.T)+t).T
        idx, dist2 = nn(src_t, dst)
        rmse = np.sqrt(dist2.mean())
        print(f"[ICP] iter {it:02d}: RMSE = {rmse:.4f} m")
        s,R,t = umeyama(src.T, dst[idx].T, with_scale=True)
        if prev_err is not None and abs(prev_err - rmse) < eps:
            print("[ICP] converged.")
            break
        prev_err = rmse
    return s,R,t, rmse

# ---------- Utilities ---------- #
def resample(k, n): return np.linspace(0,n-1,k).round().astype(int)
def rmse(a,b): return np.sqrt(((a-b)**2).sum(1).mean())

# ---------- Main pipeline ---------- #
def calibrate(slam_csv, gps_csv, use_icp=True, out_json="transform.json"):
    slam = pd.read_csv(slam_csv)[["x","y","z"]].to_numpy()
    gps  = pd.read_csv(gps_csv)[["latitude","longitude","height"]]
    lat0,lon0,alt0 = gps.iloc[0]
    enu  = lla_to_enu(gps["longitude"], gps["latitude"], gps["height"], lon0, lat0, alt0)

    k = min(len(slam),len(enu))
    if len(slam)!=k: slam = slam[resample(k,len(slam))]
    if len(enu)!=k : enu  =  enu [resample(k,len(enu))]

    # Umeyama
    s0,R0,t0 = umeyama(slam.T, enu.T, with_scale=True)
    slam_u = (s0*(R0@slam.T)+t0).T
    print(f"[Init] Umeyama RMSE = {rmse(slam_u, enu):.4f} m")

    if use_icp:
        s,R,t, final_rmse = icp_sim3(slam, enu, s0,R0,t0)
        print(f"[Done] ICP refined RMSE = {final_rmse:.4f} m")
    else:
        s,R,t = s0,R0,t0

    json_data = dict(scale=float(s), rotation=R.tolist(),
                     translation=t.flatten().tolist(),
                     lat0=float(lat0),lon0=float(lon0),alt0=float(alt0))
    Path(out_json).write_text(json.dumps(json_data,indent=2))
    print(f"[OK] transform saved → {out_json}")
    return json_data

# ---------- Online ---------- #
class SLAM2GPS:
    def __init__(self,jpath):
        cfg=json.loads(Path(jpath).read_text())
        self.s,cfg_r,cfg_t = cfg["scale"], np.array(cfg["rotation"]), np.array(cfg["translation"])
        self.R,self.t = cfg_r, cfg_t.reshape(3,1)
        self.lat0,self.lon0,self.alt0 = cfg["lat0"],cfg["lon0"],cfg["alt0"]
    def slam_to_lla(self,xyz):
        xyz=np.asarray(xyz).reshape(3,-1)
        enu=(self.s*self.R@xyz+self.t).T
        return enu_to_lla(enu[:,0],enu[:,1],enu[:,2],
                          self.lon0,self.lat0,self.alt0)

# ---------- CLI ---------- #
def _cli():
    ap=argparse.ArgumentParser(description="SLAM↔GPS w/ RMSE output & multi‑round ICP")
    ap.add_argument("slam_csv"), ap.add_argument("gps_csv")
    ap.add_argument("--no-icp",action="store_true",help="skip ICP, only Umeyama")
    ap.add_argument("-o","--output",default="transform.json")
    args=ap.parse_args()
    calibrate(args.slam_csv,args.gps_csv,not args.no_icp,args.output)

if __name__=="__main__":
    _cli()