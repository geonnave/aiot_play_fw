import mido
import pprint
import copy

pp = pprint.PrettyPrinter(indent=4,depth=2)

#============================ defines =========================================

NOTE_val2str = {}
for i in range(21,127+1):
    # notename
    if (i-21)%12==0:  notename='LA'
    if (i-21)%12==1:  notename='SIb'
    if (i-21)%12==2:  notename='SI'
    if (i-21)%12==3:  notename='DO'
    if (i-21)%12==4:  notename='REb'
    if (i-21)%12==5:  notename='RE'
    if (i-21)%12==6:  notename='MIb'
    if (i-21)%12==7:  notename='MI'
    if (i-21)%12==8:  notename='FA'
    if (i-21)%12==9:  notename='SOLb'
    if (i-21)%12==10: notename='SOL'
    if (i-21)%12==11: notename='LAb'
    # octave
    octave = int((i-12)/12)
    # name
    NOTE_val2str[i] = f'NOTE_{notename}_{octave}'

#============================ classes =========================================

class MidiParser(object):
    
    TICKS_PER_BEAT           = 120
    TEMPO                    = 500000
        
    def __init__(self,filename,maxts=None):
        
        # store params
        self.filename        = filename
        self.maxts           = maxts
        
        # initialize
        self.midi            = mido.MidiFile(f'{self.filename}.mid')
        
        # handle
        self._print_summary(self.midi)
        self.tempomap        = self._get_tempomap(self.midi)
        self.tempomapIdx     = 0
        self.miding          = self._midi2miding(self.midi)
        #self.miding          = self._remove_overlaps(self.miding)
        self.miding          = self._cut_in_track(self.miding)
        self.miding          = self._order_tracks(self.miding)
        self.miding          = self._sort_tracks(self.miding)
        self._save_miding_midi(self.miding)
        self._save_miding_h(self.miding)
        self._convert_midi_txt(f'{self.filename}')
        self._convert_midi_txt(f'{self.filename}_simplified')
    
    #======================== public ==========================================
    
    #======================== private =========================================
    
    def _print_summary(self,midi):
        output               = []
        output              += ['']
        output              += [f'Extracting tracks from "{self.filename}.mid"']
        output              += ['']
        output              += ['Initial file contains {} tracks:'.format(len(midi.tracks))]
        for (i,track) in enumerate(midi.tracks):
            output          += ['   {:>2}. {}'.format(i,track.name)]
        output              += ['']
        output               = '\n'.join(output)
        print(output)
    
    def _get_tempomap(self,midi):
        tempomap                  = []
        tempotrack                = None
        for track in midi.tracks:
            for i in range(1,len(track)):
                msg               = track[i] # shorthand
                if msg.type=='set_tempo':
                    if tempotrack==None:
                        tempotrack = track.name
                    assert(tempotrack==track.name)
                    tempomap     += [{'time':msg.time,'tempo':msg.tempo}]
        return tempomap
    
    def _midi2miding(self,midi):
        '''
        miding = [
            {
                'name': 'Bassoon',
                'notes': {
                    {'note':123,'startts':123,'endts':123},
                    {'note':123,'startts':123,'endts':123},
                    {'note':123,'startts':123,'endts':123},
                    ...
                }
            }
        ]
        '''
        miding          = []
        for track in midi.tracks:
            self.runningtime = 0
            self.runningts   = 0
            self.tempomapIdx = 0
            if track.name in ['Drums','Timpani']:
                continue
            notes            = []
            activenotes      = []
            for i in range(1,len(track)):
                msg          = track[i] # shorthand
                ts           = self.__get_ts(msg.time)
                if self.runningts>self.maxts:
                    break
                if msg.type=='note_on' and msg.velocity>0:
                    # note starts
                    assert(msg.note not in activenotes)
                    activenotes += [{'note':msg.note,'startts':ts,'endts':None}]
                if  (
                        (msg.type=='note_on' and msg.velocity==0) or
                         msg.type=='note_off'
                    ):
                    # note ends
                    found = False
                    for i in range(len(activenotes)):
                        if activenotes[i]['note']==msg.note:
                            found = True
                            activenotes[i]['endts']=ts
                            notes += [activenotes[i]]
                            activenotes.pop(i)
                            break
                    assert(found==True)
            if len(notes):
                miding += [{
                    'name':      track.name,
                    'notes':     notes,
                }]
        return miding
    
    def __get_ts(self,time):
        
        # shift on tempomap
        self.runningtime += time
        if self.runningtime>=self.tempomap[self.tempomapIdx]['time']:
            if self.tempomapIdx<len(self.tempomap)-1:
                self.tempomapIdx += 1
        
        # compute ts
        tsdiff          = self._time2ts(time)
        self.runningts += tsdiff
        
        return self.runningtime
    
    def _remove_overlaps(self,miding):
        ctr = 0
        for track in miding:
            for i in range(len(track['notes'])):
                startts = track['notes'][i]['startts']
                for j in range(len(track['notes'])):
                    if i==j:
                        continue
                    endts = track['notes'][j]['endts']
                    if 0<endts-startts and endts-startts<self._ts2time(0.050):
                        ctr += 1
                        track['notes'][j]['endts']=track['notes'][i]['startts']
        print(f'removed {ctr} overlaps')
        return miding
    
    def _cut_in_track(self,miding):
        t = 0
        while t<len(miding):
            movenotes = []
            while True:
                i           = 0
                while i<len(miding[t]['notes']):
                    j       = 0
                    moveidx = None
                    while j<len(miding[t]['notes']):
                        if i!=j:
                            if (
                                miding[t]['notes'][i]['startts']<=miding[t]['notes'][j]['startts'] and 
                                miding[t]['notes'][j]['startts']<miding[t]['notes'][i]['endts']
                               ):
                                moveidx = j
                        if moveidx!=None:
                            break
                        j += 1
                    i += 1
                    if moveidx!=None:
                        break
                if moveidx==None:
                    break
                else:
                    movenotes += [miding[t]['notes'].pop(moveidx)]
            if movenotes:
                miding += [
                    {
                        'name':  miding[t]['name']+'-',
                        'notes': movenotes,
                    }
                ]
            print('{:<20} removed {} notes'.format(miding[t]['name'],len(movenotes)))
            t += 1
        return(miding)
    
    def _order_tracks(self,miding):
        t = 0
        for t in range(len(miding)):
            miding[t]['notes'] = sorted(miding[t]['notes'],key=lambda e: e['startts'])
        return(miding)
    
    def _sort_tracks(self,miding):
        
        ontime = []
        for track in miding:
            thisontime = 0
            for note in track['notes']:
                thisontime += note['endts']-note['startts']
            ontime += [{'name':track['name'],'ontime':thisontime}]
        ontime = sorted(ontime,key=lambda e: e['ontime'],reverse=True)
        
        midingout = []
        for trackname in [e['name'] for e in ontime]:
            for t in miding:
                if t['name']==trackname:
                    midingout += [copy.deepcopy(t)]
        assert(len(midingout)==len(miding))
        
        return midingout
        
    def _save_miding_midi(self,miding):
        # one tick is 500000/120=4167us which is 239 Hz
        # on nRF52833, when using RTC, if clock at 32 kHz, 133 RTC ticks per MIDI tick
        # so use a PRESCALER=132 -> RCT clock will be running at 32kHz/(132+1)=246 Hz
        midiout              = mido.MidiFile(ticks_per_beat=self.TICKS_PER_BEAT)
        
        # meta track
        trackout             = mido.MidiTrack()
        trackout            += [mido.MetaMessage('time_signature',numerator=4, denominator=4, clocks_per_click=24, notated_32nd_notes_per_beat=8)]
        trackout            += [mido.MetaMessage('track_name',name='meta')]
        trackout            += [mido.MetaMessage('key_signature',key='C')]
        trackout            += [mido.MetaMessage('set_tempo',tempo=self.TEMPO)]
        trackout            += [mido.MetaMessage('end_of_track')]
        midiout.tracks      += [trackout]
        
        # instrument tracks
        for (channel,track) in enumerate(miding):
            trackout         = mido.MidiTrack()
            # meta
            trackout        += [mido.MetaMessage('track_name',name=track['name'])]
            trackout        += [mido.Message('program_change',channel=channel%16,program=81)]
            # gather msgs
            msgs             = []
            for n in track['notes']:
                msgs        += [{'note':n['note'],'velocity':64, 'ts':n['startts']}] # start
                msgs        += [{'note':n['note'],'velocity':0  ,'ts':n['endts']}]   # end
            # sort
            msgs             = sorted(msgs,key=lambda e: e['ts'])
            # write
            ts               = 0
            for msg in msgs:
                t            = msg['ts']-ts
                trackout    += [mido.Message('note_on',channel=channel%16,note=msg['note'],velocity=msg['velocity'],time=t)]
                ts           = msg['ts']
            # finalize
            trackout        += [mido.MetaMessage('end_of_track')]
            midiout.tracks  += [trackout]
        midiout.save(f'{self.filename}_simplified.mid')
    
    def _save_miding_h(self,miding):
        output     = []
        output    += [f'#ifndef __{self.filename}_H']
        output    += [f'#define __{self.filename}_H']
        output    += ['']
        for (trackidx,track) in enumerate(miding):
            output += [self.__format_track(trackidx,track)]
        output    += ['#endif']
        output    += ['']
        output     = '\n'.join(output)
        filename   = f'{self.filename}.h'
        with open(filename,'w') as f:
            f.write(output)
        print(f'\nCreated {self.filename}.h')
    
    def __format_track(self,trackidx,track):
        output       = []
        output      += [f'static const note_t SONGNOTES_{self.filename}_TRACK_{trackidx}[] = {{ // {track['name']}']
        laststartts  = None
        lastendts    = 0
        for note in track['notes']:
            if laststartts!=None and lastendts!=None:
                try:
                    assert(laststartts<lastendts)
                    assert(            lastendts<=note['startts'])
                    assert(                       note['startts']<note['endts'])
                except AssertionError:
                    print(laststartts,lastendts,note['startts'],note['endts'])
                    raise
            dur = note['startts']-lastendts
            if dur:
                output      += ['    {{{:<12} {}}},'.format('NOTE_NONE,',                  dur                          )]
            output          += ['    {{{:<12} {}}},'.format(NOTE_val2str[note['note']]+',',note['endts']-note['startts'])]
            laststartts      = note['startts']
            lastendts        = note['endts']
        output    += ['};']
        output    += ['']
        output     = '\n'.join(output)
        return output
    
    def _convert_midi_txt(self,filename):
        midi  = mido.MidiFile(f'{filename}.mid')
        with open(f'{filename}.txt','w') as f:
            f.write(str(midi))
    
    #======================== helpers =========================================
    
    def _ts2time(self,ts):
        # tempo: us per beat
        # ticks_per_beat in header
        # ts   = time*(tempo/ticks_per_beat)
        # time = ts/(tempo/ticks_per_beat)
        return int(ts/(self.tempomap[0]['tempo']/1000000/self.TICKS_PER_BEAT))
    
    def _time2ts(self,time):
        return time*(self.tempomap[0]['tempo']/1000000/self.TICKS_PER_BEAT)

#============================ main ============================================

def main():
    MidiParser(
        filename    = 'Star_Wars_Medley',
        maxts       = 48.0,
    )
    MidiParser(
        filename    = 'harry_potter',
        maxts       = 28.0,
    )

if __name__=='__main__':
    main()