"""
Python Wrapper of CASPR
Author: Mingrui Luo
Version: 2024/04/26
"""
import matlab.engine
import numpy as np

def li2mat(li,t=False):
    """Convert List to MATLAB Array"""
    if t==False:
        if isinstance(li,list):
            return matlab.double(li)
        else:
            return matlab.double(li.tolist())
    else:
        if isinstance(li,list):
            return matlab.double(np.array(li).T.tolist())
        else:
            return matlab.double(li.T.tolist())

class PYCASPR():
    def __init__(self,wkspace,modelname,cablename,sharemode=False):
        """Python Wrapper of CASPR"""
        print('MATLAB Workspace:',wkspace)
        if sharemode==True:
            # Share mode is generally quicker!
            self.eng = matlab.engine.connect_matlab()
        else:
            self.eng = matlab.engine.start_matlab()
            self.eng.cd(wkspace+'/..')
            self.eng.initialise_CASPR(nargout=0)
        # Init Model & Cables
        self.eng.cd(wkspace+'/PYCASPR_core')
        self.eng.PYCASPR_init(modelname,cablename,nargout=0)
    
    def Help_traj(self,trajname=None,q_p=None,q_dot=None,q_ddot=None,
                    te=None,dt=None,method_id=2):
        """Trajectory Loader or Interpolation"""
        if trajname is not None:
            # API
            q,q_dot,q_ddot,tv=self.eng.PYCASPR_traj(trajname,nargout=4)
        else:
            if q_dot is None:
                q_dot=np.zeros(q_p.shape)
            if q_ddot is None:
                q_ddot=np.zeros(q_p.shape)
            # API
            q,q_dot,q_ddot,tv=self.eng.PYCASPR_traj_gen(
            li2mat(q_p,True),li2mat(q_dot,True),li2mat(q_ddot,True),
            li2mat(np.linspace(0,te,q_p.shape[0])),dt,method_id,nargout=4)
        # Notice the array shape!
        q,q_dot,q_ddot,tv=np.array(q).T,np.array(q_dot).T,np.array(q_dot).T,tv[0]
        return q,q_dot,q_ddot,tv

    def IK_update(self,q,qd=None,qdd=None):
        """Inverse Kinematics Solver"""
        if qd is None:
            qd=np.zeros(len(q))
        if qdd is None:
            qdd=np.zeros(len(q))
        # API
        cablelengths,cableLengthsDot,segments=self.eng.PYCASPR_IK(
        li2mat(q),
        li2mat(qd),
        li2mat(qdd),nargout=3)
        return cablelengths[0],cableLengthsDot[0],segments
    
    def ID_init(self,method_id=1):
        """Initialize Inverse Dynamics Solver"""
        self.eng.PYCASPR_init_ID(method_id,nargout=0)
    
    def ID_update(self,q,qd=None,qdd=None):
        """Inverse Dynamics Solver"""
        if qd is None:
            qd=np.zeros(len(q))
        if qdd is None:
            qdd=np.zeros(len(q))
        # API
        forces,cablelengths,cableLengthsDot=self.eng.PYCASPR_ID(
        li2mat(q),
        li2mat(qd),
        li2mat(qdd),nargout=3)
        return forces[0],cablelengths[0],cableLengthsDot[0]
    
    def FK_init(self,l0,q0,q0_dot=None,method_id=2):
        """Initialize Forward Kinematics Solver"""
        if q0_dot is None:
            q0_dot=np.zeros(len(q0))
        self._data_FK_l=l0
        self._data_FK_q=q0
        self._data_FK_qd=q0_dot
        self.eng.PYCASPR_init_FK(method_id,nargout=0)
    
    def FK_update(self,l,dt=0.1):
        """Forward Kinematics Solver"""
        # API
        q,qd=self.eng.PYCASPR_FK(
            li2mat(l),li2mat(self._data_FK_l),
            li2mat(self._data_FK_q),li2mat(self._data_FK_qd),
            dt,nargout=2)
        self._data_FK_l=l
        self._data_FK_q=list(q[0])
        self._data_FK_qd=list(qd[0])
        return q[0],qd[0]
    
    def FD_init(self,q0,q0_dot=None,method_id=2):
        """Initialize Forward Dynamics Solver"""
        if q0_dot is None:
            q0_dot=np.zeros(len(q0))
        self.eng.PYCASPR_init_FD(method_id,
        li2mat(q0),li2mat(q0_dot),nargout=0)
    
    def FD_update(self,f,dt=0.1):
        """Forward Dynamics Solver"""
        # API
        q,q_dot,q_ddot,l,l_dot=self.eng.PYCASPR_FD(li2mat(f),dt,nargout=5)
        return q[0],q_dot[0],q_ddot[0],l[0],l_dot[0]