"""Bounded survey/generation lifecycle, separate from map activation. Injected I/O."""
import json


class MappingSession:
    def __init__(self, io):
        self.io=io;self.phase='IDLE';self.error='';self.mode='';self.identifier='';self.directory=''
        self.index=0;self.total=0;self.deadline=0.;self.future=None;self.snapshot={};self.world={}
        self.cancelled=False;self.finish_requested=False;self.save_requested=True;self.maps=[];self.started=0.;self.comparison=[]
        self.last_engine={};self.survey_fraction=0.

    @property
    def busy(self):return self.phase not in ('IDLE','DONE')

    def stage(self,phase,now,timeout=15.):self.phase=phase;self.deadline=now+timeout

    def request(self,mode,now):
        if self.busy:raise ValueError('A mapping session is already running')
        self.io.check_start()
        self.mode=mode;self.identifier,self.directory=self.io.new_session(mode)
        self.world=self.io.world_snapshot();self.error='';self.maps=[];self.comparison=[];self.index=0
        self.total=len(self.io.waypoints);self.started=now;self.cancelled=False
        self.finish_requested=False;self.save_requested=True;self.snapshot={}
        self.last_engine={};self.survey_fraction=0.
        self.future=self.io.pause(True);self.stage('PAUSING',now)

    def finish(self):self.finish_requested=True

    def cancel(self,now):
        if not self.busy:return
        if self.phase=='ERROR_RESUME':return  # Dispatch may have resumed; never cancel a newly assigned job.
        self.cancelled=True;self.save_requested=False
        if self.phase=='ERROR_HELD':
            self.io.role('robot1');self.io.hold(True);self.stage('STOP_ROBOT',now)
        elif self.phase not in ('PAUSING','STOP_ROBOT','CANCELLING_GOAL','STOP_MAPPERS','RETURNING_HOME','RESTORING','RELEASING','RESUMING'):
            self.io.hold(True);self.stage('STOP_ROBOT',now)

    def result(self):
        if not self.future.done():return None
        result=self.future.result()
        if not result.success:raise ValueError(result.message)
        return result

    def begin_finish(self,now):self.io.hold(True);self.stage('STOP_ROBOT',now)

    def tick(self,now):
        if not self.busy or self.phase in ('ERROR_HELD','ERROR_RESUME'):return
        try:
            engine=self.io.engine_status(self.identifier)
            if engine:self.last_engine=engine
            if now>self.deadline:
                detail=self.last_engine.get('ready_reason','No fresh mapper telemetry') if self.phase=='STARTING_MAPPERS' else ''
                raise TimeoutError(self.phase.replace('_',' ').lower()+' timed out'+(': '+detail if detail else ''))
            if self.phase=='PAUSING':
                response=self.result()
                if response is None:return
                self.snapshot=json.loads(response.message)
                self.io.role('robot1');self.io.park_world();self.stage('PARKING',now)
            elif self.phase=='PARKING':
                if not self.io.parked():return
                if self.cancelled:self.begin_finish(now);return
                self.io.start_engine(self.mode,self.identifier,self.directory)
                self.stage('STARTING_MAPPERS',now,60.)
            elif self.phase=='STARTING_MAPPERS':
                if not self.io.engine_alive():raise RuntimeError('Mapping process exited during startup')
                if not self.io.engine_status(self.identifier).get('ready'):return
                self.io.go(self.io.waypoints[0]);self.stage('SURVEYING',now,600.)
            elif self.phase=='SURVEYING':
                fraction=getattr(self.io,'goal_fraction',lambda:0.)()
                self.survey_fraction=(self.index+fraction)/max(1,self.total)
                if not self.io.engine_alive():raise RuntimeError('Mapping process exited during the survey')
                if self.finish_requested:self.begin_finish(now);return
                result=self.io.goal_result()
                if result=='FAILED':raise RuntimeError('Survey navigation failed; returning to the existing map')
                if result!='ARRIVED':return
                self.index+=1
                self.survey_fraction=self.index/max(1,self.total)
                if self.index>=self.total:self.begin_finish(now)
                else:self.io.go(self.io.waypoints[self.index]);self.stage('SURVEYING',now,600.)
            elif self.phase=='STOP_ROBOT':
                if not self.io.held():return
                self.future=self.io.cancel_goal();self.stage('CANCELLING_GOAL',now)
            elif self.phase=='CANCELLING_GOAL':
                if self.result() is None:return
                if self.save_requested and self.io.engine_alive():
                    self.future=self.io.finish_engine();self.stage('REQUESTING_SAVE',now)
                else:self.io.stop_engine();self.stage('STOP_MAPPERS',now)
            elif self.phase=='REQUESTING_SAVE':
                if self.result() is None:return
                self.stage('SAVING',now,45.)
            elif self.phase=='SAVING':
                status=self.io.engine_status(self.identifier)
                if status.get('phase') not in ('SAVED','FAILED'):return
                self.maps=status.get('maps',[])
                self.comparison=status.get('comparison',[])
                if status.get('phase')=='FAILED':self.error=status.get('error','Map generation failed')
                self.io.stop_engine();self.stage('STOP_MAPPERS',now)
            elif self.phase=='STOP_MAPPERS':
                if not self.io.engine_stopped():return
                self.io.hold(False);self.io.go('HOME_1');self.stage('RETURNING_HOME',now,600.)
            elif self.phase=='RETURNING_HOME':
                result=self.io.goal_result()
                if result=='FAILED':raise RuntimeError('Could not return the survey robot home')
                if result!='ARRIVED':return
                self.io.hold(True);self.io.restore_world(self.world);self.io.role('')
                self.stage('RESTORING',now,60.)
            elif self.phase=='RESTORING':
                if not self.io.restored(self.world):return
                self.io.hold(False);self.stage('RELEASING',now)
            elif self.phase=='RELEASING':
                if not self.io.released():return
                self.future=self.io.pause(False);self.stage('RESUMING',now)
            elif self.phase=='RESUMING':
                if self.result() is None:return
                self.phase='DONE'
        except Exception as error:
            self.error=str(error)
            if self.phase=='RESUMING':
                self.io.hold(True);self.phase='ERROR_RESUME'
                self.error+='; dispatch acknowledgement uncertain. Restart the complete demo before continuing.'
                return
            if self.phase=='PAUSING' and self.future.done():
                # A known denial has made no scene or motion changes.
                try:denied=not self.future.result().success
                except Exception:denied=False
                if denied:self.phase='DONE';return
            if self.phase in ('STARTING_MAPPERS','SURVEYING','PARKING','SAVING','REQUESTING_SAVE'):
                self.save_requested=False;self.io.hold(True);self.stage('STOP_ROBOT',now)
            else:
                self.io.hold(True);self.io.stop_engine();self.phase='ERROR_HELD'

    def status(self):
        return {'id':self.identifier,'phase':self.phase,'busy':self.busy,'mode':self.mode,
            'directory':self.directory,'waypoint':self.index,'total':self.total,'error':self.error,
            'maps':self.maps,'cancelled':self.cancelled,'started_wall':self.started,
            'comparison':self.comparison,
            'engine':self.last_engine,'survey_fraction':self.survey_fraction,
            'message':self.phase.replace('_',' ').lower()}
