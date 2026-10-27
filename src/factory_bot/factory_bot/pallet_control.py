"""One acknowledged pallet placement at a time. No ROS or GUI dependency."""
from collections import OrderedDict
import time


class PalletControl:
    def __init__(self,policy,spec,send,notify,clock=time.monotonic):
        self.policy=policy;self.spec=spec;self.send=send;self.notify=notify;self.clock=clock
        self.place='parked';self.pose=[spec['parked'][0]-policy.offset[0],spec['parked'][1]-policy.offset[1]]
        self.pending=None;self.history=OrderedDict();self.sequence=0

    def response(self,identifier,phase,message):
        self.history[identifier]={'id':identifier,'phase':phase,'message':message}
        while len(self.history)>16:self.history.popitem(last=False)

    def request(self,request,robots=None,blocked='',internal=False):
        if not isinstance(request,dict):raise ValueError('Pallet request must be an object')
        identifier=request.get('id')
        if not isinstance(identifier,str) or not 1<=len(identifier)<=80:raise ValueError('A request ID is required')
        if identifier in self.history:self.notify();return
        try:
            if self.pending:raise ValueError('Wait for the previous pallet placement acknowledgement')
            if blocked and not internal:raise ValueError(blocked)
            action=request.get('action')
            if action=='remove':
                target=[self.spec['parked'][0]-self.policy.offset[0],self.spec['parked'][1]-self.policy.offset[1]];place='parked'
            elif action=='place':
                if robots is None:raise ValueError('Waiting for fresh positions from both robots')
                valid,reason,target=self.policy.validate(request.get('x'),request.get('y'),robots)
                if not valid:raise ValueError(reason)
                place=request.get('place','grid') if internal else 'grid'
            else:raise ValueError('Choose place/move or remove')
            self.pending={'id':identifier,'started':self.clock(),'target':list(target),'place':place,'timed_out':False}
            self.response(identifier,'APPLYING','Waiting for Gazebo to confirm the pallet pose')
            self.notify()
            future=self.send(target[0]+self.policy.offset[0],target[1]+self.policy.offset[1],self.spec['z'])
            if future is None:raise RuntimeError('Gazebo set_pose service is unavailable')
            def done(f):
                try:
                    if not f.result().success:raise RuntimeError('Gazebo rejected the pallet placement')
                    self.pose=list(target);self.place=place
                    self.response(identifier,'APPLIED','Pallet parked outside' if place=='parked' else 'Pallet placed at '+str(tuple(target)))
                except Exception as error:self.response(identifier,'FAILED',str(error))
                self.pending=None;self.notify()
            future.add_done_callback(done)
        except (ValueError,RuntimeError,OSError) as error:
            # Rejected requests must not clear another in-flight placement.
            if self.pending and self.pending['id']==identifier:self.pending=None
            self.response(identifier,'REJECTED',str(error));self.notify()

    def legacy(self,place,robots=None,x=None,y=None):
        self.sequence+=1;request={'id':'internal_'+str(self.sequence),'action':'remove','place':place}
        if place!='parked':
            if place=='grid':target=self.policy.snap(x,y)
            elif place in self.spec.get('drop_points',{}):
                wx,wy=self.spec['drop_points'][place];target=(wx-self.policy.offset[0],wy-self.policy.offset[1])
            else:raise ValueError('Unknown pallet position: '+str(place))
            request.update(action='place',x=target[0],y=target[1])
        self.request(request,robots,internal=True)

    def status(self):
        if self.pending and self.clock()-self.pending['started']>8 and not self.pending['timed_out']:
            self.pending['timed_out']=True
            self.response(self.pending['id'],'UNCERTAIN','Gazebo has not confirmed the pose. Further moves are held until its reply arrives.')
        return {'place':self.place,'pose':self.pose,'pending':self.pending is not None,'requests':list(self.history.values()),
                'grid_step':self.policy.step,'footprint':list(self.policy.bounds)}
