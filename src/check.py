import numpy as np
from parts import build_all
def rot_about(h, ang):
    p1=np.array([*h[0],0.]); p2=np.array([*h[1],0.]); k=(p2-p1)/np.linalg.norm(p2-p1)
    th=np.radians(ang); K=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
    R=np.eye(3)+np.sin(th)*K+(1-np.cos(th))*K@K
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=p1-R@p1; return T
def check(p):
    fl,M=p.unfold()
    Ms=[None]*len(fl); err=0
    for i in sorted(range(len(fl)),key=lambda i:p.depth(i)):
        f=fl[i]
        Ms[i]=np.eye(4) if f['parent']<0 else Ms[f['parent']]@rot_about(f['hinge'],f['ang'])
        for q2,q3 in zip(f['poly'],p.faces[i]['pts']):
            w=M@Ms[i]@np.array([q2[0],q2[1],0,1.])
            err=max(err,np.linalg.norm(w[:3]-q3))
    return fl,err
if __name__=='__main__' and 0:
    tot=0
    for p in build_all():
        fl,err=check(p)
        xs=np.array([q for f in fl for q in f['poly']])
        bb=xs.max(0)-xs.min(0)
        m=p.mass(); n=len(p.instances); tot+=m*n
        angs=[round(f['ang']) for f in fl if f['parent']>=0]
        print(f"{p.id:7s} f={len(fl):3d} err={err:6.3f} bb={bb[0]:6.0f}x{bb[1]:5.0f} m={m*1000:6.0f}g x{n:2d} riv={len(p.rivets3d):3d} angs={angs[:8]}")
    print("sheet total kg",round(tot,1))


def main():
    worst = 0.0
    for p in build_all():
        _, err = check(p)
        worst = max(worst, err)
        if err > 0.01:
            print(f"FAIL {p.id}: refolded flat pattern is {err:.3f} mm off the 3D part")
    print(f"refold check: worst error {worst:.4f} mm over all parts")
    return 0 if worst <= 0.01 else 1


if __name__ == '__main__':
    raise SystemExit(main())
