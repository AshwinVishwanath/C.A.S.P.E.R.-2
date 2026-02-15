import { useState, useEffect, useRef, createContext, useContext } from "react";
var MASCOT_SRC="data:image/png;base64,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";;
var SAD_MASCOT_SRC="data:image/png;base64,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";
var LOGO_SRC="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAABoAAAAaCAYAAACpSkzOAAADC0lEQVR4nKXWS6jVVRQG8N+61/ftgoSFGZgpBKZW0KAizAKDLGzUwCgTclAQQYOaBEEgDRoUPSiiiaMGIQ2CaBCNgtCopg0ssaLMQshHvrt+Df775Ln3nnPuFdfk/9hrr2+tb3977V3maUnGe6/tWbhUVRky5cosSc0xPjafOHMGqapLSRbgQazGMlzEKXxdVT9dFVAfyK14Et/jtzbnHBZjM85W1Xs9/ysC6gPZgKfwQQt6A05jAcbwCW7HzVX19lxgg4AqyZIke5KsSvJKkkeTLOzzWZ/kjSQrk+xOsqWX5HxBxtpzawu+I8nWvvHxvvfJJK+2ZF4aBTToZ4/OtTiOVVX1ZZLXkmyuqqkkE0ne0a3TIazB+SQTw6gbVea/uA5Hm8RfxvZW0Y14HnfiR1zb/BcNC7ZgBNAlLNcpDHZgf1VN4WCSp7Ef97Y44zqhzBuot9N/bRn/XlVJsg/PJjmMW/BZVZ1MMtESOllVF5KMt2Sm2SzqmqwL32AS25Lc1Lj/XEfTgao6lGRSJ/tN+KLNG7hGw6irqjqdZKWOkt1JjuMMTmBDknt0XWIp1uF8q3zg3pz1s2+zvog/dcpbi2taYhcbeOEk/sIRPIy3qupIkprZbKdR1weyHYexD3/jw0bJPw2gsBAf44cG9D52tr44q4BpQA1kKZ7R7Y912IgVWKITyrd4V9f3NjTq7sAx3I0tLc54f+z/gfoGHmiZrsEeHNTJdgzf6dbpekzgfPv3BHbhzQbKZfViuhj6D7RF+KNVsk13JBzD+kbhpuY7hZ1YhbMur98sm8Zln2Ieb8CH8Qh+ab6L++acaQkd1YllP+7C3qo6PkgQQy3JmiTP9X0vSrIsyZIZfruS3D8q1sBel2Q8ycKq+hnLk2yEqrpQVWeq6lyv+iTr8SkeSrK6HTGzVDdww7YO3esQe/FCko9063AfDlbVV0ke06nyhI7KoZt2rjtDtYkrdcqawgHcpjttz1XV670Dsaoujoo30oa1lCQrZo6PujGNrGhGgGm+V3Q3uBqb67430/4D/MK0wfsvrdEAAAAASUVORK5CYII=";

const themes = {
  dark: { name:"dark",bg:"#05080c",bgEl:"#0b1018",bgPanel:"#0e1420",border:"#1c2a3c",text:"#c8d6e0",strong:"#e2e8f0",muted:"#9ab0c0",accent:"#22d3a0",accentBg:"#22d3a010",warn:"#f59e0b",danger:"#ef4444",info:"#3b82f6",shadow:"0 1px 3px rgba(0,0,0,0.5)",glow:c=>"0 0 8px "+c+"44",armedText:"#05080c",firingBg:"#f59e0b",firingText:"#05080c",gridLine:"#1a2332" },
  light: { name:"light",bg:"#e8ecf0",bgEl:"#f1f4f7",bgPanel:"#ffffff",border:"#c2cad6",text:"#334155",strong:"#0f172a",muted:"#64748b",accent:"#059669",accentBg:"#05966910",warn:"#d97706",danger:"#dc2626",info:"#2563eb",shadow:"0 1px 4px rgba(0,0,0,0.12)",glow:()=>"none",armedText:"#ffffff",firingBg:"#d97706",firingText:"#ffffff",gridLine:"#d0d8e0" },
};
const ThemeCtx = createContext(themes.dark);
const useTheme = () => useContext(ThemeCtx);
const NEVERA = "'Nevera', sans-serif";
const MONO = "'IBM Plex Mono','Menlo',monospace";
const SANS = "'IBM Plex Sans',system-ui,sans-serif";
const COND = "'IBM Plex Sans Condensed','Arial Narrow',sans-serif";
function buildTimeline(roles) {
  var hasIgnition = roles.includes("Ignition") || roles.includes("Ignition Backup");
  var hasApogee = roles.includes("Apogee") || roles.includes("Apogee Backup");
  var hasMain = roles.includes("Main") || roles.includes("Main Backup");

  var timeline = ["PAD", "BOOST"];

  if (hasIgnition) {
    // 2-stage: first coast, then sustainer ignition, then second coast
    timeline.push("COAST 1", "SUSTAIN", "COAST 2");
  } else {
    timeline.push("COAST");
  }

  // Recovery events at/after apogee
  if (hasApogee && hasMain) {
    // Dual deploy: drogue at apogee, main at altitude
    timeline.push("APOGEE", "DROGUE", "MAIN");
  } else if (hasApogee && !hasMain) {
    // Single deploy at apogee
    timeline.push("APOGEE", "RECOVERY");
  } else if (!hasApogee && hasMain) {
    // No drogue — tumble then main
    timeline.push("APOGEE", "TUMBLE", "MAIN");
  } else {
    // No recovery configured
    timeline.push("APOGEE");
  }

  timeline.push("LANDED");
  return timeline;
}
const ROLES = ["Apogee","Apogee Backup","Main","Main Backup","Ignition","Ignition Backup","Custom"];

function useSim(conn, timeline, flightActive) {
  const [d,setD] = useState({ rssi:-42,dataAge:0,batt:8.24,gpsLat:51.50741,gpsLon:-0.12784,gpsFix:"3D",gpsSats:0,ekfAlt:0,alt:0,vel:0,roll:0,pitch:90,yaw:0,mach:0,state:"PAD",t:0,apogee:0,stale:false,staleSince:0,qbar:0,integrity:100,
    pyro:[{hwCh:1,role:"Apogee",cont:false,contV:0,armed:false,firing:false},{hwCh:2,role:"Main",cont:false,contV:0,armed:false,firing:false},{hwCh:3,role:"Apogee Backup",cont:false,contV:0,armed:false,firing:false},{hwCh:4,role:"Main Backup",cont:false,contV:0,armed:false,firing:false}]
  });
  const tick = useRef(0);
  var ivRef = useRef(null);
  var delayRef = useRef(null);
  var staleRef = useRef({dropping:false,dropStart:0,dropDuration:0});
  var tlRef = useRef(timeline);
  var flightRef = useRef(flightActive);
  tlRef.current = timeline;
  flightRef.current = flightActive;

  useEffect(function() {
    if (!conn) return;
    tick.current = 0;
    staleRef.current = {dropping:false,dropStart:0,dropDuration:0};
    // When GS connects, start sending pad-idle data immediately (no 10s delay for idle)
    var iv = setInterval(function() {
      // If flight not active, show pad-idle telemetry
      if (!flightRef.current) {
        setD(function(prev){return{...prev,
          stale:false,staleSince:0,
          rssi:-42-Math.random()*8,dataAge:Math.floor(Math.random()*60+15),
          batt:8.24+Math.random()*0.03,
          gpsSats:9+Math.floor(Math.random()*5),gpsFix:"3D",
          gpsLat:51.50741+(Math.random()-0.5)*0.00002,gpsLon:-0.12784+(Math.random()-0.5)*0.00002,
          ekfAlt:0.1+Math.random()*0.3,alt:0.1+Math.random()*0.3,vel:0,
          roll:Math.random()*0.5-0.25,pitch:89.5+Math.random(),yaw:Math.random()*0.5-0.25,
          mach:0,state:"PAD",t:0,apogee:0,qbar:0,integrity:98+Math.random()*2,
          pyro:prev.pyro.map(function(p){return{...p,
            cont:p.hwCh===4?false:Math.random()>0.05,
            contV:p.hwCh===4?0.02+Math.random()*0.05:(Math.random()>0.05?2.8+Math.random()*0.4:0.02+Math.random()*0.05)
          };})
        };});
        return;
      }
      // Flight active — run the full sim
      tick.current++;
      var t = tick.current;
      var sr = staleRef.current;
      // Simulate random data drops during flight
      if (!sr.dropping && t > 50 && t < 130 && Math.random() < 0.02) {
        sr.dropping = true; sr.dropStart = t; sr.dropDuration = 8 + Math.floor(Math.random() * 20);
      }
      if (sr.dropping) {
        var elapsed = t - sr.dropStart;
        if (elapsed >= sr.dropDuration) { sr.dropping = false; }
        else {
          // During drop: only increment staleSince, zero-order hold everything else
          setD(function(prev) { return {...prev, stale:true, staleSince:prev.staleSince + 0.18, dataAge:999}; });
          return;
        }
      }
      var tl = tlRef.current;
      var hasIgn = tl.includes("SUSTAIN");
      var hasDrogue = tl.includes("DROGUE");
      var hasTumble = tl.includes("TUMBLE");
      var hasMainSt = tl.includes("MAIN");
      var hasRecovery = tl.includes("RECOVERY");
      var ph;
      if (hasIgn) {
        if(t<40)ph="PAD";else if(t<55)ph="BOOST";else if(t<65)ph="COAST 1";else if(t<78)ph="SUSTAIN";else if(t<95)ph="COAST 2";
        else if(t<100)ph="APOGEE";else if(hasDrogue&&t<110)ph="DROGUE";else if(hasTumble&&t<110)ph="TUMBLE";
        else if(hasRecovery&&t<140)ph="RECOVERY";else if(hasMainSt&&t<140)ph="MAIN";
        else ph="LANDED";
      } else {
        if(t<40)ph="PAD";else if(t<62)ph="BOOST";else if(t<90)ph="COAST";
        else if(t<95)ph="APOGEE";else if(hasDrogue&&t<105)ph="DROGUE";else if(hasTumble&&t<105)ph="TUMBLE";
        else if(hasRecovery&&t<135)ph="RECOVERY";else if(hasMainSt&&t<135)ph="MAIN";
        else ph="LANDED";
      }
      if(!tl.includes(ph)) ph=tl[tl.length-1];
      var ft=Math.max(0,t-40), alt=0, vel=0;
      if(ph==="BOOST"){alt=ft*ft*8;vel=ft*16;}
      else if(ph==="COAST"||ph==="COAST 1"){var c=Math.min(hasIgn?t-55:t-62,28);alt=2000+c*(200-c*3);vel=Math.max(0,200-c*7);}
      else if(ph==="SUSTAIN"){var c2=t-65;alt=3500+c2*c2*6;vel=c2*14+50;}
      else if(ph==="COAST 2"){var c3=t-78;alt=5000+c3*(180-c3*4);vel=Math.max(0,180-c3*8);}
      else if(ph==="APOGEE"){alt=hasIgn?7200:5150;vel=0;}
      else if(ph==="DROGUE"||ph==="TUMBLE"){var dt=hasIgn?t-100:t-95;alt=Math.max(0,(hasIgn?7200:5150)-dt*25);vel=-12-Math.random()*3;}
      else if(ph==="RECOVERY"||ph==="MAIN"){var dt2=hasIgn?t-110:t-105;alt=Math.max(0,(hasIgn?6900:4900)-dt2*30);vel=-6-Math.random()*2;}
      else if(ph==="LANDED"){alt=0;vel=0;}
      // Dynamic pressure: q = 0.5 * rho * v^2 (rho varies with altitude, simplified)
      var rho = 1.225 * Math.exp(-alt / 8500); // exponential atmosphere model
      var qbar = 0.5 * rho * vel * vel;
      // Data integrity: simulated from 3-5 repeat packets with hamming/ECC
      var baseIntegrity = 98 + Math.random() * 2;
      var machPenalty = (Math.abs(vel / 343) > 0.7 && Math.abs(vel / 343) < 1.3) ? (15 + Math.random() * 10) : 0;
      var rangePenalty = alt > 3000 ? (alt - 3000) / 500 : 0;
      var noisy = Math.random() < 0.05 ? 8 + Math.random() * 12 : 0;
      var integrity = Math.max(0, Math.min(100, baseIntegrity - machPenalty - rangePenalty - noisy));

      setD(function(prev){return{...prev,stale:false,staleSince:0,qbar:qbar,integrity:integrity,rssi:-42-Math.random()*10,dataAge:Math.floor(Math.random()*80+12),batt:8.24-t*0.0008+Math.random()*0.03,gpsSats:9+Math.floor(Math.random()*5),gpsFix:"3D",
        gpsLat:51.50741+(Math.random()-0.5)*0.00005,gpsLon:-0.12784+(Math.random()-0.5)*0.00005,
        ekfAlt:Math.max(0,alt),alt:Math.max(0,alt),vel:vel,
        roll:(ph==="BOOST"||ph==="SUSTAIN"||ph==="COAST"||ph==="COAST 1"||ph==="COAST 2")?Math.sin(t*0.28)*14:ph==="TUMBLE"?Math.sin(t*0.8)*45:Math.sin(t*0.05)*2,
        pitch:ph==="BOOST"?85:ph==="SUSTAIN"?83:(ph==="COAST"||ph==="COAST 1"||ph==="COAST 2")?75:ph==="TUMBLE"?Math.sin(t*0.5)*40+45:(ph==="MAIN"||ph==="RECOVERY")?5:90,
        yaw:(ph==="BOOST"||ph==="SUSTAIN"||ph==="COAST"||ph==="COAST 1"||ph==="COAST 2")?Math.cos(t*0.18)*5:ph==="TUMBLE"?Math.sin(t*0.6)*30:Math.random()*2,
        mach:vel/343,state:ph,t:t*100,apogee:(ph==="MAIN"||ph==="RECOVERY"||ph==="DROGUE"||ph==="TUMBLE"||ph==="LANDED")?(hasIgn?7200:5150):0,
        pyro:prev.pyro.map(function(p){return{...p,cont:p.firing?false:(p.hwCh===4?false:Math.random()>0.08),contV:p.firing?0.01:(p.hwCh===4?0.02+Math.random()*0.05:(Math.random()>0.08?2.8+Math.random()*0.4:0.02+Math.random()*0.05))};})
      };});
    }, 180);
    ivRef.current = iv;
    return function(){clearInterval(ivRef.current);};
  },[conn]);
  var toggleArm = function(i){setD(function(p){return{...p,pyro:p.pyro.map(function(c,j){return j===i?{...c,armed:!c.armed}:c;})};});};
  var firePyro = function(i){setD(function(p){return{...p,pyro:p.pyro.map(function(c,j){return j===i?{...c,firing:true}:c;})};});setTimeout(function(){setD(function(p){return{...p,pyro:p.pyro.map(function(c,j){return j===i?{...c,firing:false,cont:false,contV:0.01}:c;})};});},1200);};
  var setRole = function(i,role){setD(function(p){return{...p,pyro:p.pyro.map(function(c,j){return j===i?{...c,role:role}:c;})};});};
  return{...d,toggleArm:toggleArm,firePyro:firePyro,setRole:setRole,_tlRef:tlRef};
}

function useDiag(conn, alwaysPass) {
  var [tests,setTests] = useState([{id:"imu",label:"IMU (LSM6DSO32)",detail:"833Hz",status:"idle"},{id:"mag",label:"Magnetometer",detail:"10Hz",status:"idle"},{id:"baro",label:"Barometer",detail:"50Hz",status:"idle"},{id:"ekf",label:"EKF Init",detail:"4-state",status:"idle"},{id:"att",label:"Attitude",detail:"Comp filter",status:"idle"},{id:"flash",label:"Flash",detail:"Memory",status:"idle"},{id:"cfg",label:"Config",detail:"Hash",status:"idle"}]);
  var runAll = function(){if(!conn)return;setTests(function(p){return p.map(function(t){return{...t,status:"idle"};});});var i=0;var go=function(){if(i>=7)return;setTests(function(p){return p.map(function(t,j){return j===i?{...t,status:"running"}:t;});});setTimeout(function(){setTests(function(p){return p.map(function(t,j){return j===i?{...t,status:alwaysPass?"pass":(Math.random()>0.1?"pass":"fail")}:t;});});i++;go();},300+Math.random()*400);};go();};
  var reset = function(){setTests(function(p){return p.map(function(t){return{...t,status:"idle"};});});};
  return{tests:tests,runAll:runAll,reset:reset};
}

function Graph(props) {
  var data=props.data, color=props.color, h=props.h||130, unit=props.unit, stale=props.stale;
  var T=useTheme();
  if(!data||data.length<3) return <div style={{height:h,display:"flex",alignItems:"center",justifyContent:"center"}}><img src={MASCOT_SRC} alt="" style={{height:60,animation:"bob 2.5s ease-in-out infinite",filter:T.name==="light"?"invert(1)":"none"}} /></div>;
  var max=Math.max.apply(null,data), min=Math.min.apply(null,data), range=max-min||1, pad=10;
  var pts=data.map(function(v,i){return((i/(data.length-1))*100)+","+(pad+((max-v)/range)*(h-pad*2));}).join(" ");
  var uid="g"+color.replace("#","")+""+data.length;
  return (
    <div style={{position:"relative"}}>
      <svg viewBox={"0 0 100 "+h} style={{width:"100%",height:h,display:"block",opacity:stale?0.4:1,transition:"opacity 0.3s"}} preserveAspectRatio="none">
        <defs><linearGradient id={uid} x1="0" y1="0" x2="0" y2="1"><stop offset="0%" stopColor={color} stopOpacity={T.name==="dark"?0.2:0.12}/><stop offset="100%" stopColor={color} stopOpacity={0}/></linearGradient></defs>
        {[0,1,2,3,4].map(function(i){return <line key={i} x1="0" y1={pad+((h-pad*2)/4)*i} x2="100" y2={pad+((h-pad*2)/4)*i} stroke={T.gridLine} strokeWidth="0.3" vectorEffect="non-scaling-stroke"/>;} )}
        <polyline points={"0,"+h+" "+pts+" 100,"+h} fill={"url(#"+uid+")"} stroke="none"/>
        <polyline points={pts} fill="none" stroke={color} strokeWidth="1.8" vectorEffect="non-scaling-stroke" strokeLinejoin="round"/>
      </svg>
      {stale && <div style={{position:"absolute",inset:0,display:"flex",alignItems:"center",justifyContent:"center",pointerEvents:"none"}}><img src={SAD_MASCOT_SRC} alt="" style={{height:55,opacity:0.5,animation:"bob 3s ease-in-out infinite",filter:T.name==="light"?"invert(1)":"none"}} /></div>}
      <div style={{position:"absolute",top:4,right:4,fontFamily:MONO,fontSize:8,color:T.muted}}>{max.toFixed(0)} {unit}</div>
      <div style={{position:"absolute",bottom:4,right:4,fontFamily:MONO,fontSize:8,color:T.muted}}>{min.toFixed(0)} {unit}</div>
      <div style={{position:"absolute",top:4,left:6,fontFamily:MONO,fontSize:12,fontWeight:700,color:color}}>{data[data.length-1].toFixed(1)}<span style={{fontSize:8,fontWeight:500,color:T.muted,marginLeft:2}}>{unit}</span></div>
    </div>
  );
}

// === AUDIO UTILITIES (fully offline — Web Audio API + SpeechSynthesis) ===
var audioCtx = null;
function getAudio() {
  if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
  return audioCtx;
}
function beep(freq, dur, vol) {
  try {
    var a = getAudio();
    var o = a.createOscillator();
    var g = a.createGain();
    o.connect(g); g.connect(a.destination);
    o.frequency.value = freq || 800;
    g.gain.value = vol || 0.15;
    o.start(); o.stop(a.currentTime + (dur || 0.12));
  } catch(e) {}
}
var selectedVoiceRef = { current: null };
function speak(text, rate) {
  try {
    var u = new SpeechSynthesisUtterance(text);
    u.rate = rate || 1.1;
    u.pitch = 0.9;
    u.volume = 0.8;
    if (selectedVoiceRef.current) u.voice = selectedVoiceRef.current;
    window.speechSynthesis.cancel();
    window.speechSynthesis.speak(u);
  } catch(e) {}
}

function VoiceSelector() {
  var T = useTheme();
  var [voices, setVoices] = useState([]);
  var [selected, setSelected] = useState("");
  var [open, setOpen] = useState(false);
  var hoverTimers = useRef({});
  var dropRef = useRef(null);

  useEffect(function() {
    var load = function() {
      var v = window.speechSynthesis.getVoices();
      if (v.length > 0) {
        setVoices(v);
        var preferred = ["Microsoft Jenny", "Microsoft Aria", "Google UK English Female", "Samantha", "Daniel"];
        var best = null;
        for (var pi = 0; pi < preferred.length; pi++) {
          for (var vi = 0; vi < v.length; vi++) {
            if (v[vi].name.indexOf(preferred[pi]) >= 0) { best = v[vi]; break; }
          }
          if (best) break;
        }
        if (best) { selectedVoiceRef.current = best; setSelected(best.name); }
        else if (v.length > 0) { selectedVoiceRef.current = v[0]; setSelected(v[0].name); }
      }
    };
    load();
    window.speechSynthesis.onvoiceschanged = load;
    return function() { window.speechSynthesis.onvoiceschanged = null; };
  }, []);

  // Close on outside click
  useEffect(function() {
    if (!open) return;
    var handler = function(e) {
      if (dropRef.current && !dropRef.current.contains(e.target)) setOpen(false);
    };
    document.addEventListener("mousedown", handler);
    return function() { document.removeEventListener("mousedown", handler); };
  }, [open]);

  var pick = function(v) {
    selectedVoiceRef.current = v;
    setSelected(v.name);
    setOpen(false);
    // Clear any pending hover timers
    Object.keys(hoverTimers.current).forEach(function(k) { clearTimeout(hoverTimers.current[k]); });
  };

  var onEnter = function(v) {
    hoverTimers.current[v.name] = setTimeout(function() {
      var u = new SpeechSynthesisUtterance("Casper go for launch");
      u.voice = v;
      u.rate = 1.1; u.pitch = 0.9; u.volume = 0.8;
      window.speechSynthesis.cancel();
      window.speechSynthesis.speak(u);
    }, 3000);
  };

  var onLeave = function(v) {
    if (hoverTimers.current[v.name]) {
      clearTimeout(hoverTimers.current[v.name]);
      delete hoverTimers.current[v.name];
    }
  };

  if (voices.length === 0) return null;

  // Truncate display name
  var shortName = selected.length > 22 ? selected.substring(0, 20) + "\u2026" : selected;

  return (
    <div ref={dropRef} style={{position:"relative"}}>
      <button onClick={function(){setOpen(!open);}} style={{fontFamily:MONO,fontSize:9,padding:"3px 8px",border:"1px solid "+T.border,borderRadius:3,background:open?T.accent+"12":"transparent",color:T.muted,cursor:"pointer",display:"flex",alignItems:"center",gap:4,maxWidth:180}}>
        <span style={{overflow:"hidden",textOverflow:"ellipsis",whiteSpace:"nowrap"}}>{shortName}</span>
        <span style={{fontSize:7}}>{open?"\u25B2":"\u25BC"}</span>
      </button>
      {open && (
        <div style={{position:"absolute",top:"100%",right:0,marginTop:4,width:280,maxHeight:260,overflowY:"auto",background:T.bgEl,border:"1px solid "+T.border,borderRadius:4,zIndex:100,boxShadow:"0 4px 16px rgba(0,0,0,0.3)"}}>
          {voices.map(function(v) {
            var isSel = v.name === selected;
            return (
              <div key={v.name}
                onClick={function(){pick(v);}}
                onMouseEnter={function(){onEnter(v);}}
                onMouseLeave={function(){onLeave(v);}}
                style={{padding:"6px 10px",cursor:"pointer",background:isSel?T.accent+"15":"transparent",borderBottom:"1px solid "+T.border+"44",display:"flex",justifyContent:"space-between",alignItems:"center"}}>
                <div>
                  <div style={{fontFamily:MONO,fontSize:10,color:isSel?T.accent:T.strong,fontWeight:isSel?600:400}}>{v.name}</div>
                  <div style={{fontFamily:MONO,fontSize:8,color:T.muted}}>{v.lang}{v.localService?" \u00B7 offline":""}</div>
                </div>
                {isSel && <span style={{color:T.accent,fontSize:10}}>{"\u2713"}</span>}
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}
function toneChime() { beep(880, 0.08, 0.12); setTimeout(function(){ beep(1100, 0.12, 0.12); }, 100); }
function toneWarn() { beep(440, 0.15, 0.2); setTimeout(function(){ beep(330, 0.2, 0.2); }, 180); }
function toneFail() { beep(220, 0.3, 0.25); }

// === PRE-FLIGHT CHECKLIST ===
var CHECKS = [
  { id: "batt", label: "Battery Voltage", evalFn: function(sim, cfg) { return sim.batt >= (parseFloat(cfg.minBatt) || 7.4); }, desc: function(cfg) { return "> " + (cfg.minBatt || "7.4") + " V"; }, configKey: "minBatt", configDefault: "7.4", unit: "V" },
  { id: "gps", label: "GPS Fix", evalFn: function(sim) { return sim.gpsFix === "3D" && sim.gpsSats >= 6; }, desc: function() { return "3D fix, 6+ sats"; } },
  { id: "cont", label: "Pyro Continuity", evalFn: function(sim) { var armed = sim.pyro.filter(function(p) { return p.role && p.role !== "Custom" && p.role !== ""; }); return armed.length > 0 && armed.every(function(p) { return p.cont; }); }, desc: function() { return "All configured channels"; } },
  { id: "imu", label: "IMU Health", evalFn: function(sim) { return Math.abs(sim.pitch - 90) < 15; }, desc: function() { return "Vertical within 15\u00B0"; } },
  { id: "link", label: "Radio Link", evalFn: function(sim) { return !sim.stale && sim.dataAge < 500; }, desc: function() { return "< 500ms, not stale"; } },
  { id: "integrity", label: "Data Integrity", evalFn: function(sim, cfg) { return sim.integrity >= (parseFloat(cfg.minIntegrity) || 90); }, desc: function(cfg) { return "> " + (cfg.minIntegrity || "90") + "%"; }, configKey: "minIntegrity", configDefault: "90", unit: "%" },
];

function LaunchOps(props) {
  var T = useTheme();
  var sim = props.sim, conn = props.conn, onLaunch = props.onLaunch;
  var [audio, setAudio] = useState(true);
  var [overrides, setOverrides] = useState({});
  var [checkCfg, setCheckCfg] = useState({ minBatt: "7.4", minIntegrity: "90" });
  var [countActive, setCountActive] = useState(false);
  var [countT, setCountT] = useState(15);
  var [countHeld, setCountHeld] = useState(false);
  var [launched, setLaunched] = useState(false);
  var countRef = useRef(null);
  var lastCallout = useRef(-1);

  // Evaluate all checks
  var results = CHECKS.map(function(c) {
    var pass = conn && c.evalFn(sim, checkCfg);
    var overridden = overrides[c.id] || false;
    return { ...c, pass: pass, overridden: overridden, status: pass ? "GO" : (overridden ? "OVRD" : "NO-GO") };
  });
  var allGo = results.every(function(r) { return r.pass || r.overridden; });

  // Countdown logic
  useEffect(function() {
    if (!countActive || countHeld || launched) return;
    countRef.current = setInterval(function() {
      setCountT(function(prev) {
        var next = +(prev - 0.1).toFixed(1);
        if (next <= 0) {
          clearInterval(countRef.current);
          setLaunched(true);
          if (audio) {
            beep(1200, 0.3, 0.3);
            setTimeout(function() { speak("Ignition."); }, 400);
          }
          if (onLaunch) setTimeout(function(){ onLaunch(); }, 2000);
          return 0;
        }
        // Audio callouts
        var sec = Math.ceil(next);
        if (audio && sec !== lastCallout.current && next <= sec && next > sec - 0.15) {
          lastCallout.current = sec;
          if (sec === 15) speak("T minus 15 seconds");
          else if (sec === 10) speak("10");
          else if (sec <= 9 && sec >= 1) speak("" + sec, 1.3);
        }
        return next;
      });
    }, 100);
    return function() { clearInterval(countRef.current); };
  }, [countActive, countHeld, launched, audio]);

  // Reset on disconnect
  useEffect(function() {
    if (!conn) { setCountActive(false); setCountT(15); setLaunched(false); setCountHeld(false); lastCallout.current = -1; }
  }, [conn]);

  var toggleOverride = function(id) {
    if (audio) toneWarn();
    setOverrides(function(p) { var n = {...p}; n[id] = !n[id]; return n; });
  };

  var startCount = function() {
    if (!allGo) return;
    if (audio) { toneChime(); setTimeout(function() { speak("Casper go for launch. Terminal count. T minus 15 seconds."); }, 200); }
    setCountActive(true); setCountT(15); setLaunched(false); setCountHeld(false); lastCallout.current = -1;
  };

  var holdCount = function() {
    if (audio) { toneWarn(); speak("Hold hold hold."); }
    setCountHeld(true);
  };

  var resumeCount = function() {
    if (audio) { toneChime(); speak("Count resuming."); }
    setCountHeld(false); lastCallout.current = -1;
  };

  var abortCount = function() {
    if (audio) { toneFail(); speak("Count aborted."); }
    setCountActive(false); setCountT(15); setCountHeld(false); setLaunched(false); lastCallout.current = -1;
  };

  var fmtTime = function(t) {
    var s = Math.ceil(t);
    return "T-" + (s < 10 ? "0" : "") + s;
  };

  return (
    <div style={{animation:"fadeUp 0.18s ease-out"}}>
      <div style={{display:"flex",alignItems:"center",justifyContent:"space-between",marginBottom:14}}>
        <div>
          <h2 style={{fontFamily:SANS,fontSize:17,fontWeight:700,color:T.strong,marginBottom:2}}>Pad Preparation</h2>
          <span style={{fontFamily:MONO,fontSize:10,color:T.muted}}>Pre-flight checks + terminal count</span>
        </div>
        <div style={{display:"flex",alignItems:"center",gap:8}}>
          {audio && <VoiceSelector />}
          <button onClick={function(){setAudio(!audio);}} style={{fontFamily:MONO,fontSize:9,padding:"4px 10px",border:"1px solid "+T.border,borderRadius:3,background:audio?T.accent+"22":"transparent",color:audio?T.accent:T.muted,cursor:"pointer"}}>{audio ? "\uD83D\uDD0A ON" : "\uD83D\uDD07 OFF"}</button>
        </div>
      </div>

      {!countActive && (
        <div>
          <Panel title="Pre-Flight Checklist" accentColor={allGo ? T.accent : T.warn}>
            <div style={{display:"flex",flexDirection:"column",gap:6}}>
              {results.map(function(r) {
                return (
                  <div key={r.id} style={{display:"flex",alignItems:"center",justifyContent:"space-between",padding:"8px 10px",borderRadius:4,border:"1px solid " + (r.pass ? T.accent + "33" : r.overridden ? T.warn + "33" : T.danger + "33"),background:r.pass ? T.accent + "08" : r.overridden ? T.warn + "08" : T.danger + "08"}}>
                    <div style={{display:"flex",alignItems:"center",gap:10}}>
                      <span style={{fontFamily:MONO,fontSize:12,fontWeight:700,color:r.pass ? T.accent : r.overridden ? T.warn : T.danger,minWidth:52}}>{r.status}</span>
                      <div>
                        <div style={{fontFamily:SANS,fontSize:12,fontWeight:600,color:T.strong}}>{r.label}</div>
                        <div style={{fontFamily:MONO,fontSize:9,color:T.muted,marginTop:1}}>{r.desc(checkCfg)}{r.configKey && (
                          <span> <input className="cfg" value={checkCfg[r.configKey]} onChange={function(e){var v=e.target.value;setCheckCfg(function(p){var n={...p};n[r.configKey]=v;return n;});}} style={{width:45,display:"inline",padding:"1px 4px",fontSize:9,marginLeft:4}} />{r.unit}</span>
                        )}</div>
                      </div>
                    </div>
                    {!r.pass && (
                      <button onClick={function(){toggleOverride(r.id);}} style={{fontFamily:MONO,fontSize:8,padding:"3px 8px",border:"1px solid "+(r.overridden?T.warn:T.danger),borderRadius:3,background:r.overridden?T.warn+"22":"transparent",color:r.overridden?T.warn:T.danger,cursor:"pointer",letterSpacing:0.5}}>{r.overridden?"CANCEL OVRD":"OVERRIDE"}</button>
                    )}
                  </div>
                );
              })}
            </div>

            <div style={{marginTop:16,padding:"12px 0",borderTop:"1px solid "+T.border,display:"flex",alignItems:"center",justifyContent:"space-between"}}>
              <div style={{display:"flex",alignItems:"center",gap:10}}>
                <span style={{width:14,height:14,borderRadius:"50%",background:allGo?T.accent:T.danger,boxShadow:allGo?T.glow(T.accent):T.glow(T.danger)}}/>
                <span style={{fontFamily:SANS,fontSize:14,fontWeight:700,color:allGo?T.accent:T.danger}}>{allGo ? "CASPER GO FOR LAUNCH" : "NO-GO — RESOLVE ITEMS"}</span>
              </div>
            </div>
          </Panel>

          <div style={{marginTop:16,display:"flex",justifyContent:"center"}}>
            <button onClick={startCount} disabled={!allGo || !conn} style={{fontFamily:MONO,fontSize:14,fontWeight:700,letterSpacing:2,padding:"16px 48px",border:"2px solid "+(allGo&&conn?T.danger:T.border),borderRadius:6,background:allGo&&conn?T.danger+"18":"transparent",color:allGo&&conn?T.danger:T.muted,cursor:allGo&&conn?"pointer":"not-allowed",opacity:allGo&&conn?1:0.4,transition:"all 0.2s"}}>{"\u25B6 TERMINAL COUNT"}</button>
          </div>
        </div>
      )}

      {countActive && (
        <div style={{display:"flex",flexDirection:"column",alignItems:"center",gap:20}}>
          <Panel title={launched ? "LAUNCH" : countHeld ? "HOLD" : "TERMINAL COUNT"} accentColor={launched ? T.accent : countHeld ? T.warn : T.danger}>
            <div style={{textAlign:"center",padding:"20px 0"}}>
              <div style={{fontFamily:MONO,fontSize:64,fontWeight:700,color:launched?T.accent:countHeld?T.warn:countT<=10?T.danger:T.strong,letterSpacing:4,textShadow:launched?"0 0 20px "+T.accent:countT<=10?"0 0 20px "+T.danger:"none",transition:"color 0.2s"}}>{launched ? "LAUNCH" : fmtTime(countT)}</div>
              {!launched && <div style={{fontFamily:MONO,fontSize:11,color:T.muted,marginTop:6}}>{countHeld ? "Count is held" : "Terminal count in progress"}</div>}
              {launched && <div style={{fontFamily:MONO,fontSize:11,color:T.accent,marginTop:6}}>Casper is away</div>}
            </div>

            {!launched && (
              <div style={{display:"flex",gap:10,justifyContent:"center",marginTop:10}}>
                {!countHeld && <button onClick={holdCount} style={{fontFamily:MONO,fontSize:11,fontWeight:700,padding:"10px 28px",border:"2px solid "+T.warn,borderRadius:4,background:T.warn+"18",color:T.warn,cursor:"pointer",letterSpacing:1}}>HOLD</button>}
                {countHeld && <button onClick={resumeCount} style={{fontFamily:MONO,fontSize:11,fontWeight:700,padding:"10px 28px",border:"2px solid "+T.accent,borderRadius:4,background:T.accent+"18",color:T.accent,cursor:"pointer",letterSpacing:1}}>RESUME</button>}
                <button onClick={abortCount} style={{fontFamily:MONO,fontSize:11,fontWeight:700,padding:"10px 28px",border:"2px solid "+T.danger,borderRadius:4,background:T.danger+"18",color:T.danger,cursor:"pointer",letterSpacing:1}}>ABORT</button>
              </div>
            )}
            {launched && (
              <div style={{display:"flex",justifyContent:"center",marginTop:10}}>
                <button onClick={abortCount} style={{fontFamily:MONO,fontSize:10,padding:"6px 20px",border:"1px solid "+T.border,borderRadius:3,background:"transparent",color:T.muted,cursor:"pointer"}}>RESET</button>
              </div>
            )}
          </Panel>

          {!launched && (
            <div style={{width:"100%",maxWidth:400}}>
              <div style={{display:"grid",gridTemplateColumns:"1fr 1fr 1fr",gap:6}}>
                {results.map(function(r) {
                  return (
                    <div key={r.id} style={{textAlign:"center",padding:"6px 4px",borderRadius:3,border:"1px solid "+(r.pass||r.overridden?T.accent+"33":T.danger+"33"),background:r.pass||r.overridden?T.accent+"08":T.danger+"08"}}>
                      <div style={{fontFamily:MONO,fontSize:8,fontWeight:700,color:r.pass||r.overridden?T.accent:T.danger}}>{r.status}</div>
                      <div style={{fontFamily:MONO,fontSize:7,color:T.muted,marginTop:1}}>{r.label}</div>
                    </div>
                  );
                })}
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
}

function RadarScope(props) {
  var T = useTheme();
  var canvasRef = useRef(null);
  var propsRef = useRef(props);
  propsRef.current = props;
  var stateRef = useRef({ angle: 0, trail: [], lastRecord: 0, maxRSmooth: 100 });
  var themeRef = useRef(T);
  themeRef.current = T;

  useEffect(function() {
    var running = true;
    var draw = function() {
      if (!running) return;
      var cv = canvasRef.current;
      if (!cv) { requestAnimationFrame(draw); return; }
      var ctx = cv.getContext("2d");
      var p = propsRef.current;
      var Th = themeRef.current;
      var dpr = Math.min(window.devicePixelRatio || 1, 2);
      var size = Math.min(cv.clientWidth, cv.clientHeight);
      if (size < 10) { requestAnimationFrame(draw); return; }
      cv.width = size * dpr;
      cv.height = size * dpr;
      ctx.scale(dpr, dpr);
      var cx = size / 2, cy = size / 2, R = size / 2 - 14;
      var st = stateRef.current;
      var dk = Th.name === "dark";
      var gR = dk ? "34,211,160" : "5,150,105";
      var gHex = dk ? "#22d3a0" : "#059669";
      var ga = function(a) { return "rgba(" + gR + "," + a + ")"; };
      var bg = dk ? "#080d12" : "#e2e6ea";
      var mono = "IBM Plex Mono";

      // GPS delta to meters
      var dx = 0, dy = 0, dist = 0;
      if (p.connected && p.padLat && p.padLon) {
        dy = (p.rocketLat - p.padLat) * 111320;
        dx = (p.rocketLon - p.padLon) * 111320 * Math.cos(p.padLat * Math.PI / 180);
        dist = Math.sqrt(dx * dx + dy * dy);
      }

      // Dynamic range - smoothly adapts, scales down for close targets
      var ranges = [10, 25, 50, 100, 250, 500, 1000, 2500, 5000, 10000];
      var targetR = 100;
      for (var i = 0; i < ranges.length; i++) {
        if (dist < ranges[i] * 0.7) { targetR = ranges[i]; break; }
        if (i === ranges.length - 1) targetR = ranges[i];
      }
      st.maxRSmooth += (targetR - st.maxRSmooth) * 0.02;
      var maxR = st.maxRSmooth;

      // Advance sweep (~3.5s per revolution)
      st.angle = (st.angle + 0.03) % (Math.PI * 2);

      // Record position into trail (throttled: every 300ms)
      var now = Date.now();
      if (p.connected && dist > 0.1 && now - st.lastRecord > 300) {
        var sweepAngle = Math.atan2(dy, dx);
        if (sweepAngle < 0) sweepAngle += Math.PI * 2;
        st.trail.push({ x: dx, y: dy, sa: sweepAngle, revAt: -1, br: 0 });
        st.lastRecord = now;
        if (st.trail.length > 120) st.trail.shift();
      }

      // Reveal blips when sweep passes over their angle
      for (var bi = 0; bi < st.trail.length; bi++) {
        var b = st.trail[bi];
        var ad = st.angle - b.sa;
        while (ad < 0) ad += Math.PI * 2;
        while (ad > Math.PI * 2) ad -= Math.PI * 2;
        if (ad < 0.1) {
          b.br = 1.0;
          b.revAt = now;
        }
        if (b.revAt > 0) {
          b.br = Math.max(0, 1.0 - (now - b.revAt) / 4000);
        }
      }
      st.trail = st.trail.filter(function(b) { return b.revAt < 0 || b.br > 0.01; });

      // === DRAW ===
      ctx.clearRect(0, 0, size, size);
      ctx.save();
      ctx.beginPath();
      ctx.arc(cx, cy, R + 1, 0, Math.PI * 2);
      ctx.clip();

      ctx.fillStyle = bg;
      ctx.fillRect(0, 0, size, size);

      // Sweep glow wedge
      if (ctx.createConicGradient) {
        ctx.save();
        var sg = ctx.createConicGradient(-st.angle, cx, cy);
        sg.addColorStop(0, ga(dk ? 0.07 : 0.05));
        sg.addColorStop(0.08, ga(dk ? 0.03 : 0.02));
        sg.addColorStop(0.18, ga(0));
        sg.addColorStop(1, ga(0));
        ctx.fillStyle = sg;
        ctx.beginPath();
        ctx.arc(cx, cy, R, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }

      // Range rings
      for (var ri = 1; ri <= 4; ri++) {
        var rr = R * ri / 4;
        ctx.beginPath();
        ctx.arc(cx, cy, rr, 0, Math.PI * 2);
        ctx.strokeStyle = ri === 4 ? ga(0.22) : ga(0.08);
        ctx.lineWidth = ri === 4 ? 1.2 : 0.5;
        ctx.stroke();
        var lbl = maxR * ri / 4;
        var ls = lbl >= 1000 ? (lbl / 1000).toFixed(1) + "km" : lbl < 10 ? lbl.toFixed(1) + "m" : Math.round(lbl) + "m";
        ctx.fillStyle = ga(0.25);
        ctx.font = "8px " + mono;
        ctx.textAlign = "center";
        ctx.fillText(ls, cx, cy - rr + 10);
      }

      // Crosshairs
      ctx.strokeStyle = ga(0.06);
      ctx.lineWidth = 0.5;
      ctx.beginPath(); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.stroke();

      // Cardinals
      ctx.fillStyle = ga(0.35);
      ctx.font = "bold 9px " + mono;
      ctx.textAlign = "center";
      ctx.fillText("N", cx, cy - R + 20);
      ctx.fillText("S", cx, cy + R - 12);
      ctx.textAlign = "left"; ctx.fillText("E", cx + R - 16, cy + 3);
      ctx.textAlign = "right"; ctx.fillText("W", cx - R + 16, cy + 3);

      // Sweep line
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + Math.cos(st.angle) * R, cy - Math.sin(st.angle) * R);
      ctx.strokeStyle = ga(0.55);
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Trail blips - only visible after sweep reveals them
      for (var j = 0; j < st.trail.length; j++) {
        var tb = st.trail[j];
        if (tb.br < 0.02) continue;
        var bx = cx + (tb.x / maxR) * R;
        var by = cy - (tb.y / maxR) * R;
        var bd = Math.sqrt((bx - cx) * (bx - cx) + (by - cy) * (by - cy));
        if (bd > R) continue;
        ctx.beginPath();
        ctx.arc(bx, by, 1.5 + tb.br * 2, 0, Math.PI * 2);
        ctx.fillStyle = ga(tb.br * 0.5);
        ctx.fill();
        if (tb.br > 0.6) {
          ctx.beginPath();
          ctx.arc(bx, by, 5, 0, Math.PI * 2);
          ctx.fillStyle = ga(tb.br * 0.08);
          ctx.fill();
        }
      }

      // Current position (always visible)
      if (p.connected && dist > 0.1) {
        var rx = cx + (dx / maxR) * R;
        var ry = cy - (dy / maxR) * R;
        var rd = Math.sqrt((rx - cx) * (rx - cx) + (ry - cy) * (ry - cy));
        if (rd <= R) {
          ctx.beginPath(); ctx.arc(rx, ry, 7, 0, Math.PI * 2);
          ctx.fillStyle = ga(0.12); ctx.fill();
          ctx.beginPath(); ctx.arc(rx, ry, 2.5, 0, Math.PI * 2);
          ctx.fillStyle = gHex; ctx.fill();
        }
      }

      // Center (GS)
      ctx.beginPath(); ctx.arc(cx, cy, 2, 0, Math.PI * 2);
      ctx.fillStyle = gHex; ctx.fill();
      ctx.beginPath(); ctx.arc(cx, cy, 4.5, 0, Math.PI * 2);
      ctx.strokeStyle = ga(0.3); ctx.lineWidth = 0.7; ctx.stroke();

      // CRT scanlines
      ctx.fillStyle = dk ? "rgba(0,0,0,0.04)" : "rgba(255,255,255,0.025)";
      for (var sl = 0; sl < size; sl += 3) { ctx.fillRect(0, sl, size, 1); }

      ctx.restore();

      // Distance/bearing readout
      if (p.connected && dist > 0.1) {
        var brg = Math.atan2(dx, dy) * 180 / Math.PI;
        if (brg < 0) brg += 360;
        var ds = dist >= 1000 ? (dist / 1000).toFixed(2) + "km" : dist.toFixed(0) + "m";
        ctx.fillStyle = ga(0.65);
        ctx.font = "bold 9px " + mono;
        ctx.textAlign = "left";
        ctx.fillText(ds + "  " + brg.toFixed(0) + "\u00B0", 8, size - 5);
      }

      requestAnimationFrame(draw);
    };
    requestAnimationFrame(draw);
    return function() { running = false; };
  }, []);

  return (
    <div style={{position:"relative",width:"100%",height:"100%",minHeight:200}}>
      <canvas ref={canvasRef} style={{width:"100%",height:"100%",minHeight:200,display:"block",borderRadius:4}} />
    </div>
  );
}

function RocketCanvas(props) {
  var roll=props.roll, pitch=props.pitch, yaw=props.yaw;
  var T=useTheme();
  var ref=useRef(null);
  useEffect(function(){
    var cv=ref.current; if(!cv)return;
    var ctx=cv.getContext("2d");
    var dpr=Math.min(window.devicePixelRatio||1,2);
    var W=cv.clientWidth, H=cv.clientHeight;
    cv.width=W*dpr; cv.height=H*dpr; ctx.scale(dpr,dpr);
    var cx=W/2, cy=H/2, sc=55;
    var dr=function(d){return d*Math.PI/180;};
    var rr=dr(roll), pp=dr(pitch-90), yy=dr(yaw);
    var rY=function(p,a){var c=Math.cos(a),s=Math.sin(a);return[p[0]*c+p[2]*s,p[1],-p[0]*s+p[2]*c];};
    var rX=function(p,a){var c=Math.cos(a),s=Math.sin(a);return[p[0],p[1]*c-p[2]*s,p[1]*s+p[2]*c];};
    var rZ=function(p,a){var c=Math.cos(a),s=Math.sin(a);return[p[0]*c-p[1]*s,p[0]*s+p[1]*c,p[2]];};
    var xf=function(p){return rZ(rX(rY(p,yy),pp),rr);};
    var pj=function(p){var d2=5,f=d2/(d2-p[2]*0.3);return[cx+p[0]*sc*f,cy-p[1]*sc*f];};
    var pt=function(p){return pj(xf(p));};
    ctx.clearRect(0,0,W,H);
    // Grid
    ctx.strokeStyle=T.gridLine;ctx.lineWidth=0.5;ctx.globalAlpha=0.4;
    for(var i=-5;i<=5;i++){var s=i/5*1.5;var a=pt([s,-1.5,-1.5]),b=pt([s,-1.5,1.5]);ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();var c2=pt([-1.5,-1.5,s]),d2=pt([1.5,-1.5,s]);ctx.beginPath();ctx.moveTo(c2[0],c2[1]);ctx.lineTo(d2[0],d2[1]);ctx.stroke();}
    ctx.globalAlpha=1;
    // Axes
    var o=pt([0,0,0]);
    [[1.2,0,0,"#ff4444"],[0,1.2,0,"#44ee66"],[0,0,1.2,"#4488ff"]].forEach(function(ax){var e=pt([ax[0],ax[1],ax[2]]);ctx.strokeStyle=ax[3];ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(o[0],o[1]);ctx.lineTo(e[0],e[1]);ctx.stroke();});
    // Body
    var br2=0.12, segs=12;
    var ring=function(y,r){var pts2=[];for(var ii=0;ii<segs;ii++){var aa=(ii/segs)*Math.PI*2;pts2.push([Math.cos(aa)*r,y,Math.sin(aa)*r]);}return pts2;};
    var topR=ring(1,br2), botR=ring(-1,br2);
    var bodyCol=T.name==="dark"?"#8899bb":"#556688";
    // Fill
    ctx.fillStyle=bodyCol;ctx.globalAlpha=0.12;ctx.beginPath();
    topR.forEach(function(p,ii){var q=pt(p);if(ii===0)ctx.moveTo(q[0],q[1]);else ctx.lineTo(q[0],q[1]);});
    var botRev=[...botR].reverse();
    botRev.forEach(function(p){var q=pt(p);ctx.lineTo(q[0],q[1]);});
    ctx.fill();ctx.globalAlpha=1;
    // Wireframe
    ctx.strokeStyle=bodyCol;ctx.lineWidth=1;
    [topR,botR].forEach(function(r){ctx.beginPath();r.forEach(function(p,ii){var q=pt(p);if(ii===0)ctx.moveTo(q[0],q[1]);else ctx.lineTo(q[0],q[1]);});var q=pt(r[0]);ctx.lineTo(q[0],q[1]);ctx.stroke();});
    for(var ii=0;ii<segs;ii+=2){var aa=pt(topR[ii]),bb=pt(botR[ii]);ctx.beginPath();ctx.moveTo(aa[0],aa[1]);ctx.lineTo(bb[0],bb[1]);ctx.stroke();}
    // Nose
    ctx.strokeStyle=T.name==="dark"?"#ee5533":"#cc3311";ctx.lineWidth=1.2;
    var nt=pt([0,1.5,0]);
    for(var ii2=0;ii2<segs;ii2+=3){var pp2=pt(topR[ii2]);ctx.beginPath();ctx.moveTo(nt[0],nt[1]);ctx.lineTo(pp2[0],pp2[1]);ctx.stroke();}
    // Fins
    ctx.strokeStyle=T.name==="dark"?"#55bb88":"#338866";ctx.lineWidth=1.5;
    for(var fi=0;fi<4;fi++){var fa=(fi/4)*Math.PI*2,fco=Math.cos(fa),fsi=Math.sin(fa);var f1=pt([fco*br2,-0.6,fsi*br2]),f2=pt([fco*0.35,-1.1,fsi*0.35]),f3=pt([fco*br2,-1.1,fsi*br2]);ctx.beginPath();ctx.moveTo(f1[0],f1[1]);ctx.lineTo(f2[0],f2[1]);ctx.lineTo(f3[0],f3[1]);ctx.closePath();ctx.stroke();}
  },[roll,pitch,yaw,T.name]);
  return (
    <div style={{position:"relative",width:"100%",height:"100%",minHeight:200}}>
      <canvas ref={ref} style={{width:"100%",height:"100%",minHeight:200,display:"block"}}/>
      <div style={{position:"absolute",bottom:6,left:8,fontFamily:MONO,fontSize:9,color:T.muted,background:T.bgPanel+"dd",padding:"3px 8px",borderRadius:3}}>
        <span style={{color:"#ff6655"}}>{"φ "+roll.toFixed(1)+"°"}</span>{"  "}<span style={{color:"#55ee88"}}>{"θ "+pitch.toFixed(1)+"°"}</span>{"  "}<span style={{color:"#5588ff"}}>{"ψ "+yaw.toFixed(1)+"°"}</span>
      </div>
    </div>
  );
}

function Dot(props){var T=useTheme();var c=props.status==="ok"?T.accent:props.status==="warn"?T.warn:props.status==="fail"?T.danger:T.muted;return <span style={{width:7,height:7,borderRadius:"50%",background:c,display:"inline-block",boxShadow:T.glow(c),animation:props.status==="ok"?"none":"pulse 1.6s infinite",flexShrink:0}}/>;}

function Panel(props){var T=useTheme();var bc=props.accentColor||T.border;return(<div style={{background:T.bgPanel,border:"1px solid "+bc,borderRadius:5,overflow:"hidden",boxShadow:T.shadow,...(props.style||{})}}>{props.title&&<div style={{padding:"7px 12px",borderBottom:"1px solid "+bc,display:"flex",alignItems:"center",justifyContent:"space-between",background:props.accentColor?props.accentColor+"08":T.bgEl}}><span style={{fontFamily:COND,fontSize:10.5,fontWeight:600,color:props.accentColor||T.muted,textTransform:"uppercase",letterSpacing:1.8}}>{props.title}</span>{props.right}</div>}<div style={{padding:"10px 12px"}}>{props.children}</div></div>);}

function Btn(props){var T=useTheme();return <button onClick={props.onClick} disabled={props.disabled} style={{fontFamily:MONO,fontSize:10.5,fontWeight:700,letterSpacing:0.8,padding:"6px 14px",borderRadius:3,border:props.primary?"none":"1px solid "+T.border,background:props.primary?(props.disabled?T.muted:T.accent):"transparent",color:props.primary?(T.name==="dark"?"#05080c":"#fff"):T.text,cursor:props.disabled?"not-allowed":"pointer",opacity:props.disabled?0.45:1}}>{props.children}</button>;}

function CfgRow(props){var T=useTheme();return <div style={{display:"flex",alignItems:"center",gap:8,minHeight:28}}><label style={{fontFamily:MONO,fontSize:10.5,color:T.muted,width:200,flexShrink:0,lineHeight:1.3}}>{props.label}</label><div style={{display:"flex",alignItems:"center",gap:5,flex:1}}>{props.children}{props.unit&&<span style={{fontFamily:MONO,fontSize:9,color:T.muted,whiteSpace:"nowrap"}}>{props.unit}</span>}</div></div>;}

function Toggle(props){var T=useTheme();return <div style={{display:"inline-flex",borderRadius:3,border:"1px solid "+T.border,overflow:"hidden"}}>{props.options.map(function(o){return <button key={o.value} onClick={function(){props.onChange(o.value);}} style={{fontFamily:MONO,fontSize:9,fontWeight:700,padding:"3px 8px",border:"none",background:props.value===o.value?T.accent:"transparent",color:props.value===o.value?(T.name==="dark"?"#05080c":"#fff"):T.muted,cursor:"pointer",letterSpacing:0.5}}>{o.label}</button>;})}</div>;}

function FlightStateBar(props){var T=useTheme();var tl=props.timeline;var idx=tl.indexOf(props.current);var sc={PAD:T.muted,BOOST:T.danger,"COAST":T.info,"COAST 1":T.info,"COAST 2":T.info,SUSTAIN:T.warn,APOGEE:T.accent,DROGUE:T.accent,TUMBLE:T.danger,RECOVERY:T.accent,MAIN:T.warn,LANDED:T.accent};return <div style={{display:"flex",background:T.bgPanel,border:"1px solid "+T.border,borderRadius:5,overflow:"hidden",boxShadow:T.shadow}}>{tl.map(function(s,i){var a=i===idx,p=i<idx,c=sc[s]||T.muted;return <div key={s} style={{flex:1,padding:"8px 4px",textAlign:"center",position:"relative",background:a?c+"20":p?c+"08":"transparent",borderRight:i<tl.length-1?"1px solid "+T.border:"none"}}><div style={{position:"absolute",top:0,left:0,right:0,height:3,background:a?c:p?c+"60":"transparent"}}/><div style={{fontFamily:COND,fontSize:10,fontWeight:700,letterSpacing:1.5,color:a?c:p?c+"88":T.muted}}>{s}</div></div>;})}</div>;}

function VerticalStateBar(props){
  var T=useTheme();var tl=props.timeline;var idx=tl.indexOf(props.current);
  var sc={PAD:T.muted,BOOST:T.danger,"COAST":T.info,"COAST 1":T.info,"COAST 2":T.info,SUSTAIN:T.warn,APOGEE:T.accent,DROGUE:T.accent,TUMBLE:T.danger,RECOVERY:T.accent,MAIN:T.warn,LANDED:T.accent};
  // Reverse so PAD is at bottom, LANDED at top (like altitude)
  var reversed = tl.slice().reverse();
  return <div style={{display:"flex",flexDirection:"column",background:T.bgPanel,border:"1px solid "+T.border,borderRadius:5,overflow:"hidden",boxShadow:T.shadow,height:"100%"}}>
    {reversed.map(function(s,ri){var i=tl.length-1-ri;var a=i===idx,p=i<idx,c=sc[s]||T.muted;
      return <div key={s} style={{flex:1,display:"flex",alignItems:"center",justifyContent:"center",position:"relative",background:a?c+"20":p?c+"08":"transparent",borderBottom:ri<tl.length-1?"1px solid "+T.border:"none",minHeight:28}}>
        <div style={{position:"absolute",left:0,top:0,bottom:0,width:3,background:a?c:p?c+"60":"transparent"}}/>
        <div style={{fontFamily:COND,fontSize:7.5,fontWeight:700,letterSpacing:1,color:a?c:p?c+"88":T.muted,writingMode:"vertical-rl",textOrientation:"mixed",transform:"rotate(180deg)"}}>{s}</div>
      </div>;
    })}
  </div>;
}

function MiniTerminalCount(props){
  var T=useTheme();var sim=props.sim,conn=props.conn,allGo=props.allGo,onLaunch=props.onLaunch;
  var [active,setActive]=useState(false);
  var [countT,setCountT]=useState(15);
  var [held,setHeld]=useState(false);
  var [launched,setLaunched]=useState(false);
  var countRef=useRef(null);
  var lastCall=useRef(-1);

  useEffect(function(){
    if(!active||held||launched)return;
    countRef.current=setInterval(function(){
      setCountT(function(prev){
        var next=+(prev-0.1).toFixed(1);
        if(next<=0){clearInterval(countRef.current);setLaunched(true);beep(1200,0.3,0.3);setTimeout(function(){speak("Ignition.");},300);
          // Trigger flight sim
          if(onLaunch)setTimeout(function(){onLaunch();},1500);
          return 0;}
        var sec=Math.ceil(next);
        if(sec!==lastCall.current&&next<=sec&&next>sec-0.15){
          lastCall.current=sec;
          if(sec===15)speak("T minus 15 seconds");
          else if(sec===10)speak("10");
          else if(sec<=9&&sec>=1)speak(""+sec,1.3);
        }
        return next;
      });
    },100);
    return function(){clearInterval(countRef.current);};
  },[active,held,launched]);

  useEffect(function(){if(!conn){setActive(false);setCountT(15);setLaunched(false);setHeld(false);lastCall.current=-1;}},[conn]);

  var start=function(){if(!allGo)return;toneChime();speak("Casper go for launch. Terminal count. T minus 15 seconds.");setActive(true);setCountT(15);setLaunched(false);setHeld(false);lastCall.current=-1;};
  var abort=function(){toneWarn();setActive(false);setCountT(15);setHeld(false);setLaunched(false);lastCall.current=-1;};

  if(!active){
    return <div style={{display:"flex",justifyContent:"center"}}>
      <button onClick={start} disabled={!allGo} style={{fontFamily:MONO,fontSize:11,fontWeight:700,letterSpacing:1.5,padding:"14px 20px",border:"2px solid "+(allGo?T.danger:T.border),borderRadius:5,background:allGo?T.danger+"15":"transparent",color:allGo?T.danger:T.muted,cursor:allGo?"pointer":"not-allowed",opacity:allGo?1:0.4,width:"100%"}}>▶ TERMINAL<br/>COUNT</button>
    </div>;
  }

  return <Panel title={launched?"IGNITION":held?"HOLD":"T-COUNT"} accentColor={launched?T.accent:T.danger}>
    <div style={{textAlign:"center"}}>
      <div style={{fontFamily:MONO,fontSize:36,fontWeight:700,color:launched?T.accent:countT<=5?T.danger:T.strong,letterSpacing:2}}>{launched?"GO":"T-"+(Math.ceil(countT)<10?"0":"")+Math.ceil(countT)}</div>
      {!launched&&<div style={{display:"flex",gap:4,justifyContent:"center",marginTop:8}}>
        <button onClick={function(){setHeld(!held);}} style={{fontFamily:MONO,fontSize:8,fontWeight:700,padding:"3px 8px",border:"1px solid "+T.warn,borderRadius:3,background:T.warn+"15",color:T.warn,cursor:"pointer"}}>{held?"RESUME":"HOLD"}</button>
        <button onClick={abort} style={{fontFamily:MONO,fontSize:8,fontWeight:700,padding:"3px 8px",border:"1px solid "+T.danger,borderRadius:3,background:T.danger+"15",color:T.danger,cursor:"pointer"}}>ABORT</button>
      </div>}
      {launched&&<button onClick={abort} style={{fontFamily:MONO,fontSize:8,padding:"3px 8px",border:"1px solid "+T.border,borderRadius:3,background:"transparent",color:T.muted,cursor:"pointer",marginTop:6}}>RESET</button>}
    </div>
  </Panel>;
}

function PyroBox(props){
  var T=useTheme();var pyro=props.pyro,cont=pyro.cont,contV=pyro.contV,armed=pyro.armed,firing=pyro.firing,role=pyro.role;
  var bg,tc,brc;
  if(firing){bg=T.firingBg;tc=T.firingText;brc=T.warn;}else if(armed&&cont){bg=T.accent;tc=T.armedText;brc=T.accent;}else if(armed&&!cont){bg=T.danger;tc=T.armedText;brc=T.danger;}else{bg="transparent";tc=T.strong;brc=T.border;}
  var filled=armed||firing;
  return(
    <div style={{flex:1,minWidth:140,borderRadius:6,border:"2.5px solid "+brc,background:bg,padding:"12px 10px",display:"flex",flexDirection:"column",alignItems:"center",gap:5,position:"relative",overflow:"hidden",boxShadow:filled?"0 0 16px "+brc+"44":"none"}}>
      {firing&&<div style={{position:"absolute",inset:0,background:"repeating-linear-gradient(45deg,transparent,transparent 8px,"+T.warn+"22 8px,"+T.warn+"22 16px)",animation:"stripeMove 0.4s linear infinite"}}/>}
      <div style={{position:"relative",zIndex:1,width:"100%",textAlign:"center"}}>
        <div style={{fontFamily:MONO,fontSize:11,fontWeight:700,color:filled?tc:T.strong}}>{role}</div>
        <div style={{fontFamily:COND,fontSize:8.5,color:filled?tc+"99":T.muted,letterSpacing:1.5,marginTop:2}}>HW CH {pyro.hwCh}</div>
      </div>
      <div style={{fontFamily:COND,fontSize:11,fontWeight:700,letterSpacing:2.5,color:firing?tc:armed?tc:T.muted,position:"relative",zIndex:1}}>{firing?"FIRING":armed?"ARMED":"DISARMED"}</div>
      <div style={{fontFamily:MONO,fontSize:17,fontWeight:700,color:filled?tc:(cont?T.accent:T.danger),position:"relative",zIndex:1}}>{contV.toFixed(2)}<span style={{fontSize:9,fontWeight:500}}> V</span></div>
      <div style={{fontFamily:MONO,fontSize:8.5,fontWeight:600,color:filled?tc+"bb":(cont?T.accent:T.danger),position:"relative",zIndex:1}}>{cont?"CONTINUITY OK":"NO CONTINUITY"}</div>
      <button onClick={props.onToggleArm} disabled={firing} style={{fontFamily:MONO,fontSize:9.5,fontWeight:700,letterSpacing:1,padding:"4px 0",borderRadius:3,width:"100%",marginTop:3,cursor:firing?"not-allowed":"pointer",border:filled?"1px solid "+(T.name==="dark"?"rgba(255,255,255,0.25)":"rgba(0,0,0,0.2)"):"1px solid "+T.border,background:filled?"rgba("+(T.name==="dark"?"255,255,255":"0,0,0")+",0.1)":"transparent",color:filled?tc:T.text,position:"relative",zIndex:1}}>{armed?"DISARM":"ARM"}</button>
      {armed&&!firing&&<button onClick={props.onFire} style={{fontFamily:MONO,fontSize:9.5,fontWeight:700,letterSpacing:1,padding:"4px 0",borderRadius:3,width:"100%",border:"none",background:cont?T.warn:T.muted,color:cont?"#000":T.text,cursor:cont?"pointer":"not-allowed",opacity:cont?1:0.5,position:"relative",zIndex:1,animation:"fadeUp 0.15s ease-out"}}>▶ FIRE</button>}
    </div>
  );
}

function RoleConfig(props) {
  var T=useTheme(), role=props.role, hwCh=props.hwCh, cfg=props.cfg, setCfg=props.setCfg, imperial=props.imperial;
  var pfx="p"+hwCh+"_";
  var v=function(key){return cfg[pfx+key]||"";};
  var set=function(key,val){setCfg(function(prev){var n={...prev};n[pfx+key]=val;return n;});};
  var distU=imperial?"ft":"m", velU=imperial?"ft/s":"m/s";
  var sensorToggle = <CfgRow label="Altitude Source"><Toggle value={v("sensor")||"ekf"} onChange={function(val){set("sensor",val);}} options={[{value:"ekf",label:"MULTI-SENSOR (EKF)"},{value:"baro",label:"BARO ONLY"}]}/></CfgRow>;
  var fireDuration = <CfgRow label="Channel Fire Duration" unit="s"><input className="cfg" value={v("duration")||"1"} onChange={function(e){set("duration",e.target.value);}} style={{maxWidth:80}}/></CfgRow>;
  var secHead = function(label,color){return <div style={{fontFamily:COND,fontSize:9.5,fontWeight:700,color:color||T.accent,textTransform:"uppercase",letterSpacing:1.5,borderBottom:"1px solid "+T.border,paddingBottom:3,marginTop:4}}>{label}</div>;};

  if(role==="Apogee") return <div style={{display:"flex",flexDirection:"column",gap:9}}>{sensorToggle}{fireDuration}</div>;
  if(role==="Apogee Backup") return <div style={{display:"flex",flexDirection:"column",gap:9}}>{sensorToggle}{fireDuration}<CfgRow label="Time After Apogee" unit="s"><input className="cfg" value={v("time_after")||"1"} onChange={function(e){set("time_after",e.target.value);}} style={{maxWidth:80}}/></CfgRow></div>;
  if(role==="Main") return (
    <div style={{display:"flex",flexDirection:"column",gap:9}}>
      {sensorToggle}
      <CfgRow label="Deploy Altitude" unit={distU}><input className="cfg" value={v("deploy_alt")||"700"} onChange={function(e){set("deploy_alt",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Deploy Early if Too Fast?"><Toggle value={v("early_deploy")||"no"} onChange={function(val){set("early_deploy",val);}} options={[{value:"yes",label:"YES"},{value:"no",label:"NO"}]}/></CfgRow>
      {v("early_deploy")==="yes"&&<CfgRow label="  Velocity Threshold" unit={velU}><input className="cfg" value={v("early_vel")||"30"} onChange={function(e){set("early_vel",e.target.value);}} style={{maxWidth:80}}/></CfgRow>}
      {fireDuration}
    </div>
  );
  if(role==="Main Backup") {
    var bm=v("backup_mode")||"time";
    return (
      <div style={{display:"flex",flexDirection:"column",gap:9}}>
        {sensorToggle}
        <CfgRow label="Deploy Altitude" unit={distU}><input className="cfg" value={v("deploy_alt")||"700"} onChange={function(e){set("deploy_alt",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
        {fireDuration}
        <CfgRow label="Backup Trigger Mode"><Toggle value={bm} onChange={function(val){set("backup_mode",val);}} options={[{value:"time",label:"TIME AFTER MAIN"},{value:"height",label:"HEIGHT BELOW MAIN"}]}/></CfgRow>
        {bm==="time"?<CfgRow label="  Time After Main" unit="s"><input className="cfg" value={v("backup_time")||"1"} onChange={function(e){set("backup_time",e.target.value);}} style={{maxWidth:80}}/></CfgRow>:<CfgRow label="  Height Below Main Alt" unit={distU}><input className="cfg" value={v("backup_height")||"100"} onChange={function(e){set("backup_height",e.target.value);}} style={{maxWidth:80}}/></CfgRow>}
      </div>
    );
  }
  if(role==="Ignition") return (
    <div style={{display:"flex",flexDirection:"column",gap:9}}>
      <CfgRow label="After Motor Number"><input className="cfg" value={v("motor_num")||"1"} onChange={function(e){set("motor_num",e.target.value);}} style={{maxWidth:60}}/></CfgRow>
      <CfgRow label="Min Velocity" unit={velU}><input className="cfg" value={v("min_vel")||"50"} onChange={function(e){set("min_vel",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Min Altitude" unit={distU}><input className="cfg" value={v("min_alt")||"500"} onChange={function(e){set("min_alt",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Max Ignition Angle" unit="°"><input className="cfg" value={v("max_ign_angle")||"15"} onChange={function(e){set("max_ign_angle",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Max Flight Angle" unit="°"><input className="cfg" value={v("max_flt_angle")||"30"} onChange={function(e){set("max_flt_angle",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      {fireDuration}
    </div>
  );
  if(role==="Ignition Backup") return (
    <div style={{display:"flex",flexDirection:"column",gap:9}}>
      <CfgRow label="After Motor Number"><input className="cfg" value={v("motor_num")||"1"} onChange={function(e){set("motor_num",e.target.value);}} style={{maxWidth:60}}/></CfgRow>
      <CfgRow label="Min Velocity" unit={velU}><input className="cfg" value={v("min_vel")||"50"} onChange={function(e){set("min_vel",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Min Altitude" unit={distU}><input className="cfg" value={v("min_alt")||"500"} onChange={function(e){set("min_alt",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Max Ignition Angle" unit="°"><input className="cfg" value={v("max_ign_angle")||"15"} onChange={function(e){set("max_ign_angle",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Max Flight Angle" unit="°"><input className="cfg" value={v("max_flt_angle")||"30"} onChange={function(e){set("max_flt_angle",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      {fireDuration}
      <CfgRow label="Firing Time Delay" unit="s"><input className="cfg" value={v("fire_delay")||"1"} onChange={function(e){set("fire_delay",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
    </div>
  );
  // Custom
  var bm2=v("backup_mode")||"time";
  return (
    <div style={{display:"flex",flexDirection:"column",gap:9}}>
      {secHead("Altitude Source")}
      {sensorToggle}
      {secHead("Trigger")}
      <CfgRow label="After Motor Number"><input className="cfg" value={v("motor_num")||"1"} onChange={function(e){set("motor_num",e.target.value);}} style={{maxWidth:60}}/></CfgRow>
      <CfgRow label="Min Velocity" unit={velU}><input className="cfg" value={v("min_vel")||"0"} onChange={function(e){set("min_vel",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Min Altitude" unit={distU}><input className="cfg" value={v("min_alt")||"0"} onChange={function(e){set("min_alt",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Deploy Altitude" unit={distU}><input className="cfg" value={v("deploy_alt")||"0"} onChange={function(e){set("deploy_alt",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Time After Apogee" unit="s"><input className="cfg" value={v("time_after")||"0"} onChange={function(e){set("time_after",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      {secHead("Safety",T.warn)}
      <CfgRow label="Max Ignition Angle" unit="°"><input className="cfg" value={v("max_ign_angle")||"90"} onChange={function(e){set("max_ign_angle",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Max Flight Angle" unit="°"><input className="cfg" value={v("max_flt_angle")||"90"} onChange={function(e){set("max_flt_angle",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
      <CfgRow label="Deploy Early if Too Fast?"><Toggle value={v("early_deploy")||"no"} onChange={function(val){set("early_deploy",val);}} options={[{value:"yes",label:"YES"},{value:"no",label:"NO"}]}/></CfgRow>
      {v("early_deploy")==="yes"&&<CfgRow label="  Velocity Threshold" unit={velU}><input className="cfg" value={v("early_vel")||"30"} onChange={function(e){set("early_vel",e.target.value);}} style={{maxWidth:80}}/></CfgRow>}
      <CfgRow label="Backup Trigger Mode"><Toggle value={bm2} onChange={function(val){set("backup_mode",val);}} options={[{value:"time",label:"TIME"},{value:"height",label:"HEIGHT"}]}/></CfgRow>
      {bm2==="time"?<CfgRow label="  Backup Time" unit="s"><input className="cfg" value={v("backup_time")||"0"} onChange={function(e){set("backup_time",e.target.value);}} style={{maxWidth:80}}/></CfgRow>:<CfgRow label="  Backup Height" unit={distU}><input className="cfg" value={v("backup_height")||"0"} onChange={function(e){set("backup_height",e.target.value);}} style={{maxWidth:80}}/></CfgRow>}
      {secHead("Firing")}
      {fireDuration}
      <CfgRow label="Firing Time Delay" unit="s"><input className="cfg" value={v("fire_delay")||"0"} onChange={function(e){set("fire_delay",e.target.value);}} style={{maxWidth:80}}/></CfgRow>
    </div>
  );
}

function DiagTab(props) {
  var T=useTheme(), conn=props.conn, alwaysPass=props.alwaysPass, diag=useDiag(conn, alwaysPass);
  var pass=diag.tests.filter(function(t){return t.status==="pass";}).length;
  var fail=diag.tests.filter(function(t){return t.status==="fail";}).length;
  var total=diag.tests.length;
  return (
    <div style={{animation:"fadeUp 0.18s ease-out"}}>
      <div style={{display:"flex",alignItems:"center",justifyContent:"space-between",marginBottom:14}}>
        <div>
          <h2 style={{fontFamily:SANS,fontSize:17,fontWeight:700,color:T.strong,marginBottom:2}}>Sensor & System Diagnostics</h2>
          <span style={{fontFamily:MONO,fontSize:10,color:T.muted}}>{pass}/{total} passed{fail>0?" · "+fail+" failed":""}</span>
        </div>
        <div style={{display:"flex",gap:8}}><Btn onClick={diag.reset}>RESET</Btn><Btn primary disabled={!conn} onClick={diag.runAll}>▶ RUN ALL</Btn></div>
      </div>
      {(pass+fail>0)&&<div style={{height:3,borderRadius:2,background:T.border,marginBottom:14,overflow:"hidden"}}><div style={{height:"100%",borderRadius:2,transition:"width 0.3s",width:((pass+fail)/total*100)+"%",background:fail>0?"linear-gradient(90deg,"+T.accent+","+T.danger+")":T.accent}}/></div>}
      <div style={{display:"grid",gridTemplateColumns:"1fr 1fr 1fr",gap:10}}>
        {diag.tests.map(function(t){
          var icon=t.status==="pass"?"✓":t.status==="fail"?"✗":t.status==="running"?"⟳":"○";
          var c=t.status==="pass"?T.accent:t.status==="fail"?T.danger:t.status==="running"?T.info:T.muted;
          return <Panel key={t.id} accentColor={t.status==="pass"?T.accent:t.status==="fail"?T.danger:undefined}><div style={{display:"flex",alignItems:"center",gap:10}}><span style={{fontFamily:MONO,fontSize:18,color:c,fontWeight:700,animation:t.status==="running"?"spin 1s linear infinite":"none"}}>{icon}</span><div><div style={{fontFamily:SANS,fontSize:13,color:T.strong,fontWeight:600}}>{t.label}</div><div style={{fontFamily:MONO,fontSize:10,color:T.muted,marginTop:2}}>{t.detail}</div></div></div></Panel>;
        })}
      </div>
    </div>
  );
}

export default function CasperMC() {
  var [mode,setMode] = useState("dark");
  var T = themes[mode];
  var [tab,setTab] = useState("setup");
  var [fcConn,setFcConn] = useState(false); // USB to flight computer (setup only)
  var [gsConn,setGsConn] = useState(false); // GS connected to MC (radio link)
  var [flightActive,setFlightActive] = useState(false); // True after terminal count
  var [checklistOpen,setChecklistOpen] = useState(false); // Expandable checklist overlay
  var [padPos,setPadPos] = useState({lat:51.50741,lon:-0.12784});
  var [imperial,setImperial] = useState(true);
  var defaultTL = buildTimeline(["Apogee","Main","Apogee Backup","Main Backup"]);
  // Sim gets flightActive — before terminal count it shows pad-idle data, after it runs the flight
  var sim = useSim(gsConn, defaultTL, flightActive);
  var usedRoles=sim.pyro.map(function(p){return p.role;});
  var timeline = buildTimeline(usedRoles);
  sim._tlRef && (sim._tlRef.current = timeline);
  var [altH,setAltH] = useState([]);
  var [velH,setVelH] = useState([]);
  var [qbarH,setQbarH] = useState([]);
  var [intH,setIntH] = useState([]);
  var [cfg,setCfg] = useState(function(){
    var init = {};
    [1,2,3,4].forEach(function(h){init["p"+h+"_duration"]="1";init["p"+h+"_sensor"]="ekf";});
    return init;
  });
  var [uploadDone,setUploadDone] = useState(false);
  // Checklist state (shared between overlay and mini display)
  var [overrides, setOverrides] = useState({});
  var [checkCfg, setCheckCfg] = useState({ minBatt: "7.4", minIntegrity: "90" });

  // Auto-connect to FC when on setup tab
  useEffect(function(){
    if(tab==="setup"){
      var t=setTimeout(function(){setFcConn(true);},800);
      return function(){clearTimeout(t);};
    }
  },[tab]);
  useEffect(function(){
    if(tab!=="setup") setFcConn(false);
  },[tab]);

  useEffect(function(){
    if(!gsConn)return;
    setAltH(function(p){return p.slice(-150).concat([sim.alt]);});
    setVelH(function(p){return p.slice(-150).concat([sim.vel]);});
    setQbarH(function(p){return p.slice(-150).concat([sim.qbar]);});
    setIntH(function(p){return p.slice(-150).concat([sim.integrity]);});
  },[sim.alt,gsConn]);

  // Flight state TTS callouts
  var prevStateRef = useRef("PAD");
  useEffect(function(){
    if(!flightActive||!gsConn) return;
    var prev = prevStateRef.current;
    var cur = sim.state;
    if(prev===cur) return;
    prevStateRef.current = cur;
    // State transition callouts
    if(prev==="BOOST"&&(cur==="COAST"||cur==="COAST 1")){
      speak("Motor burnout.");
    } else if(prev==="COAST 1"&&cur==="SUSTAIN"){
      beep(800,0.15,0.2);
      setTimeout(function(){speak("Second stage ignition confirmed.");},200);
    } else if(cur==="APOGEE"){
      toneChime();
      setTimeout(function(){speak("Apogee detected.");},200);
    } else if(cur==="DROGUE"){
      speak("Drogue parachute deployed.");
    } else if(cur==="MAIN"||cur==="RECOVERY"){
      speak("Main parachute deployed.");
    } else if(cur==="TUMBLE"){
      toneFail();
      setTimeout(function(){speak("Warning. Tumble detected. No drogue deployment.");},200);
    } else if(cur==="LANDED"){
      toneChime();
      setTimeout(function(){speak("The rocket has landed.");},300);
    }
  },[sim.state,flightActive,gsConn]);

  var m2ft=function(v){return v*3.28084;};
  var altVal=function(v){return imperial?m2ft(v):v;};
  var altU=imperial?"ft":"m";
  var velVal=function(v){return imperial?m2ft(v):v;};
  var velU=imperial?"ft/s":"m/s";
  var tabs=[{id:"setup",label:"SETUP",icon:"⚙"},{id:"flight",label:"FLIGHT",icon:"▲"},{id:"tracking",label:"TRACK",icon:"◎"}];

  var onLaunch = function(){
    setFlightActive(true);
    setChecklistOpen(false);
  };

  var handleUpload = function(){
    setUploadDone(true);
  };

  // Checklist evaluation
  var checkResults = CHECKS.map(function(c) {
    var pass = gsConn && c.evalFn(sim, checkCfg);
    var overridden = overrides[c.id] || false;
    return { ...c, pass: pass, overridden: overridden, status: pass ? "GO" : (overridden ? "OVRD" : "NO-GO") };
  });
  var allGo = checkResults.every(function(r) { return r.pass || r.overridden; });

  var toggleOverride = function(id) {
    toneWarn();
    setOverrides(function(p) { var n = {...p}; n[id] = !n[id]; return n; });
  };

  return (
    <ThemeCtx.Provider value={T}>
      <div style={{width:"100%",height:"100vh",background:T.bg,fontFamily:SANS,color:T.text,display:"flex",flexDirection:"column",overflow:"hidden"}}>
        <style>{"\
          @import url('https://fonts.cdnfonts.com/css/nevera-132852');\
          @import url('https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600;700&family=IBM+Plex+Sans:wght@400;500;600;700&family=IBM+Plex+Sans+Condensed:wght@400;500;600;700&display=swap');\
          *{box-sizing:border-box;margin:0;padding:0}\
          ::-webkit-scrollbar{width:5px}::-webkit-scrollbar-track{background:"+T.bg+"}::-webkit-scrollbar-thumb{background:"+T.border+";border-radius:3px}\
          @keyframes pulse{0%,100%{opacity:1}50%{opacity:0.3}}\
          @keyframes spin{from{transform:rotate(0deg)}to{transform:rotate(360deg)}}\
          @keyframes fadeUp{from{opacity:0;transform:translateY(6px)}to{opacity:1;transform:translateY(0)}}\
          @keyframes stripeMove{from{background-position:0 0}to{background-position:22.6px 0}}\
          @keyframes bob{0%,100%{transform:translateY(0px)}50%{transform:translateY(-8px)}}\
          input.cfg,select.cfg{background:"+T.bg+";border:1px solid "+T.border+";color:"+T.strong+";font-family:"+MONO+";font-size:11px;padding:5px 8px;border-radius:3px;outline:none;width:100%}\
          input.cfg:focus,select.cfg:focus{border-color:"+T.accent+"}\
          select.cfg{cursor:pointer}\
        "}</style>

        {/* HEADER */}
        <div style={{display:"flex",alignItems:"center",justifyContent:"space-between",padding:"0 16px",height:48,borderBottom:"1px solid "+T.border,background:T.bgEl,flexShrink:0}}>
          <div style={{display:"flex",alignItems:"center",gap:10}}>
            <img src={LOGO_SRC} alt="Casper" style={{width:28,height:28,filter:T.name==="light"?"invert(1)":"none"}} />
            <span style={{fontFamily:NEVERA,fontWeight:400,fontSize:16,color:T.accent,letterSpacing:2.5,textTransform:"uppercase",fontFeatureSettings:"'salt' 0, 'ss01' 0"}}>Casper</span>
            <span style={{fontFamily:COND,fontSize:10,color:T.muted,letterSpacing:1.5,fontWeight:500}}>MISSION CONTROL</span>
            <div style={{display:"flex",alignItems:"center",gap:12,marginLeft:8}}>
              <div style={{display:"flex",alignItems:"center",gap:4}}>
                <span style={{width:7,height:7,borderRadius:"50%",background:fcConn?"#38bdf8":T.muted+"55",boxShadow:fcConn?T.glow("#38bdf8"):"none"}}/>
                <span style={{fontFamily:MONO,fontSize:9,fontWeight:600,color:fcConn?"#38bdf8":T.muted}}>FC</span>
              </div>
              <div style={{display:"flex",alignItems:"center",gap:4}}>
                <span style={{width:7,height:7,borderRadius:"50%",background:gsConn?T.accent:T.muted+"55",boxShadow:gsConn?T.glow(T.accent):"none",animation:gsConn?"none":"pulse 1.6s infinite"}}/>
                <span style={{fontFamily:MONO,fontSize:9,fontWeight:600,color:gsConn?T.accent:T.muted}}>GS</span>
              </div>
            </div>
          </div>
          <div style={{display:"flex",alignItems:"center",gap:10}}>
            <button onClick={function(){setMode(function(m){return m==="dark"?"light":"dark";});}} style={{fontFamily:MONO,fontSize:10,padding:"4px 10px",borderRadius:3,border:"1px solid "+T.border,background:T.bgPanel,color:T.text,cursor:"pointer",fontWeight:600}}>{mode==="dark"?"☀":"●"}</button>
            <button onClick={function(){setImperial(!imperial);}} style={{fontFamily:MONO,fontSize:10,padding:"4px 10px",borderRadius:3,border:"1px solid "+T.border,background:T.bgPanel,color:T.accent,cursor:"pointer",fontWeight:700}}>{imperial?"FT":"M"}</button>
            <Btn primary onClick={function(){setGsConn(!gsConn);if(gsConn){setFlightActive(false);setAltH([]);setVelH([]);setQbarH([]);setIntH([]);}}}>{gsConn?"GS DISCONNECT":"GS CONNECT"}</Btn>
          </div>
        </div>

        {/* BODY: sidebar + content */}
        <div style={{display:"flex",flex:1,overflow:"hidden"}}>

          {/* VERTICAL TAB SIDEBAR */}
          <div style={{width:72,borderRight:"1px solid "+T.border,background:T.bgEl,display:"flex",flexDirection:"column",paddingTop:8,flexShrink:0}}>
            {tabs.map(function(t){
              var active = tab===t.id;
              return <button key={t.id} onClick={function(){setTab(t.id);}} style={{
                display:"flex",flexDirection:"column",alignItems:"center",gap:3,padding:"12px 4px",border:"none",cursor:"pointer",
                background:active?T.accentBg:"transparent",
                borderLeft:active?"3px solid "+T.accent:"3px solid transparent",
                color:active?T.accent:T.muted,transition:"all 0.15s"
              }}>
                <span style={{fontSize:16}}>{t.icon}</span>
                <span style={{fontFamily:COND,fontSize:8,fontWeight:700,letterSpacing:1.5}}>{t.label}</span>
              </button>;
            })}
          </div>

          {/* MAIN CONTENT */}
          <div style={{flex:1,overflow:"auto",padding:14,position:"relative"}}>

            {/* ===== SETUP TAB ===== */}
            {tab==="setup"&&(
              <div style={{animation:"fadeUp 0.18s ease-out",maxWidth:1400,margin:"0 auto"}}>
                <div style={{display:"flex",alignItems:"center",justifyContent:"space-between",marginBottom:16}}>
                  <div>
                    <h2 style={{fontFamily:SANS,fontSize:18,fontWeight:700,color:T.strong,marginBottom:2}}>Setup & Configuration</h2>
                    <span style={{fontFamily:MONO,fontSize:10,color:fcConn?"#38bdf8":T.muted}}>{fcConn?"● USB connected to Flight Computer"+(uploadDone?" · Config uploaded":""):"Waiting for USB connection to Flight Computer..."}</span>
                  </div>
                  <div style={{display:"flex",gap:8}}>
                    <Btn>DEFAULTS</Btn><Btn>EXPORT</Btn><Btn primary disabled={!fcConn} onClick={handleUpload}>{uploadDone?"✓ UPLOADED":"▲ UPLOAD TO FC"}</Btn>
                  </div>
                </div>
                <DiagTab conn={fcConn} alwaysPass={true}/>
                <div style={{marginTop:20}}>
                  <div style={{fontFamily:COND,fontSize:11,fontWeight:700,color:T.muted,textTransform:"uppercase",letterSpacing:2,marginBottom:10}}>Pyro Channel Configuration</div>
                  <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:12}}>
                    {sim.pyro.map(function(p,i){
                      var avail=ROLES.filter(function(r){return r===p.role||!usedRoles.includes(r);});
                      return <Panel key={p.hwCh} title={<span style={{display:"flex",alignItems:"center",gap:8}}>
                        <select value={p.role} onChange={function(e){sim.setRole(i,e.target.value);}} className="cfg" style={{width:"auto",maxWidth:180,fontSize:12,fontWeight:700,padding:"3px 8px",fontFamily:MONO}}>
                          {avail.map(function(r){return <option key={r} value={r}>{r}</option>;})}
                        </select>
                      </span>} right={<span style={{fontFamily:MONO,fontSize:9,color:T.muted}}>HW CH {p.hwCh}</span>}>
                        <RoleConfig role={p.role} hwCh={p.hwCh} cfg={cfg} setCfg={setCfg} imperial={imperial}/>
                      </Panel>;
                    })}
                  </div>
                </div>
              </div>
            )}

            {/* ===== FLIGHT TAB ===== */}
            {tab==="flight"&&(
              <div style={{animation:"fadeUp 0.18s ease-out",display:"flex",gap:12,maxWidth:1400,margin:"0 auto"}}>

                <div style={{flex:1,display:"flex",flexDirection:"column",gap:10,minWidth:0}}>

                  {/* Status panels row */}
                  <div style={{display:"grid",gridTemplateColumns:"1fr 1.3fr 1fr 0.8fr",gap:10}}>
                    <Panel title="GPS" style={{cursor:"pointer"}}>
                      <div style={{display:"flex",alignItems:"center",gap:6}}>
                        <Dot status={gsConn&&sim.gpsFix==="3D"?"ok":gsConn?"warn":"fail"}/>
                        <span style={{fontFamily:MONO,fontSize:13,fontWeight:700,color:gsConn?(sim.gpsFix==="3D"?T.accent:T.warn):T.muted}}>{gsConn?sim.gpsFix+" · "+sim.gpsSats+" sats":"---"}</span>
                      </div>
                      <div style={{fontFamily:MONO,fontSize:10,color:T.text,marginTop:2}}>{gsConn?sim.gpsLat.toFixed(5)+"°N  "+Math.abs(sim.gpsLon).toFixed(5)+"°W":""}</div>
                    </Panel>
                    <Panel title="EKF Altitude">
                      <div style={{fontFamily:MONO,fontSize:22,fontWeight:700,color:T.strong}}>{gsConn?altVal(sim.ekfAlt).toFixed(1):"---"}<span style={{fontSize:10,fontWeight:500,color:T.muted,marginLeft:3}}>{altU}</span></div>
                    </Panel>
                    <Panel title="Radio Link" accentColor={sim.stale?T.warn:undefined} style={{cursor:"pointer"}}>
                      <div style={{fontFamily:MONO,fontSize:16,fontWeight:700,color:gsConn?(sim.stale?T.warn:sim.dataAge>500?T.danger:sim.dataAge>200?T.warn:T.strong):T.muted}}>{gsConn?(sim.stale?"STALE! "+sim.staleSince.toFixed(1)+"s":sim.dataAge+" ms"):"---"}</div>
                      <div style={{fontFamily:MONO,fontSize:9,marginTop:2,color:sim.stale?T.warn:T.muted}}>{gsConn?(sim.stale?"Zero-order hold":sim.rssi.toFixed(0)+" dBm"):""}</div>
                    </Panel>
                    <Panel title="Battery">
                      <div style={{fontFamily:MONO,fontSize:16,fontWeight:700,color:gsConn?(sim.batt<7.2?T.danger:sim.batt<7.6?T.warn:T.strong):T.muted}}>{gsConn?sim.batt.toFixed(2)+" V":"---"}</div>
                    </Panel>
                  </div>

                  {/* Pyro channels */}
                  <div>
                    <div style={{fontFamily:COND,fontSize:10.5,fontWeight:700,color:T.muted,textTransform:"uppercase",letterSpacing:1.8,marginBottom:6,paddingLeft:2}}>Pyro Channels</div>
                    <div style={{display:"flex",gap:10}}>
                      {sim.pyro.map(function(p,i){return <PyroBox key={p.hwCh} pyro={p} onToggleArm={function(){sim.toggleArm(i);}} onFire={function(){sim.firePyro(i);}}/>;} )}
                    </div>
                  </div>

                  {/* Graphs row: Altitude | Center (checklist+count) | Velocity */}
                  <div style={{display:"grid",gridTemplateColumns:"1fr auto 1fr",gap:10,alignItems:"start"}}>
                    <Panel title="Altitude"><Graph data={altH.map(function(v2){return altVal(v2);})} color={T.accent} h={280} unit={altU} stale={sim.stale}/></Panel>

                    {/* Center column: clickable pre-flight summary + terminal count */}
                    <div style={{display:"flex",flexDirection:"column",gap:8,width:200}}>
                      <div onClick={function(){if(!flightActive)setChecklistOpen(true);}} style={{cursor:flightActive?"default":"pointer"}}>
                        <Panel title="Pre-Flight" accentColor={allGo?T.accent:gsConn?T.warn:undefined}>
                          <div style={{display:"flex",flexDirection:"column",gap:4}}>
                            {checkResults.map(function(r){
                              return <div key={r.id} style={{display:"flex",alignItems:"center",gap:6}}>
                                <span style={{fontFamily:MONO,fontSize:9,fontWeight:700,color:r.pass?T.accent:r.overridden?T.warn:T.danger,width:36}}>{r.status}</span>
                                <span style={{fontFamily:MONO,fontSize:8,color:T.muted}}>{r.label}</span>
                              </div>;
                            })}
                          </div>
                          {!flightActive&&<div style={{fontFamily:MONO,fontSize:7,color:T.muted,textAlign:"center",marginTop:6,opacity:0.6}}>Click to expand</div>}
                        </Panel>
                      </div>
                      <MiniTerminalCount sim={sim} conn={gsConn} allGo={allGo} onLaunch={onLaunch}/>
                    </div>

                    <Panel title="Velocity"><Graph data={velH.map(function(v2){return velVal(v2);})} color={T.info} h={280} unit={velU} stale={sim.stale}/></Panel>
                  </div>

                  {/* Bottom row: q̄ + 3D orientation */}
                  <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:10}}>
                    <Panel title="Dynamic Pressure (q̄)"><Graph data={qbarH.map(function(v2){return v2/1000;})} color={T.warn} h={140} unit="kPa" stale={sim.stale}/></Panel>
                    <Panel title="3D Orientation" style={{overflow:"hidden"}}><div style={{margin:"-10px -12px"}}><RocketCanvas roll={sim.roll} pitch={sim.pitch} yaw={sim.yaw}/></div></Panel>
                  </div>
                </div>

                {/* RIGHT: vertical flight state bar */}
                <div style={{width:56,flexShrink:0}}>
                  <VerticalStateBar current={gsConn?sim.state:"PAD"} timeline={timeline}/>
                </div>
              </div>
            )}

            {/* ===== TRACKING TAB ===== */}
            {tab==="tracking"&&(
              <div style={{animation:"fadeUp 0.18s ease-out",maxWidth:1400,margin:"0 auto"}}>
                <div style={{display:"flex",alignItems:"center",justifyContent:"space-between",marginBottom:14}}>
                  <div>
                    <h2 style={{fontFamily:SANS,fontSize:18,fontWeight:700,color:T.strong,marginBottom:2}}>Tracking & Orientation</h2>
                    <span style={{fontFamily:MONO,fontSize:10,color:T.muted}}>3D attitude + ground track radar</span>
                  </div>
                </div>
                <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:12}}>
                  <Panel title="3D Orientation" style={{overflow:"hidden"}}><div style={{margin:"-10px -12px",minHeight:350}}><RocketCanvas roll={sim.roll} pitch={sim.pitch} yaw={sim.yaw}/></div></Panel>
                  <Panel title="Ground Track" style={{overflow:"hidden"}}><div style={{margin:"-10px -12px",minHeight:350}}><RadarScope rocketLat={sim.gpsLat} rocketLon={sim.gpsLon} padLat={padPos.lat} padLon={padPos.lon} connected={gsConn}/></div></Panel>
                </div>
              </div>
            )}

            {/* ===== CHECKLIST OVERLAY ===== */}
            {checklistOpen&&(
              <div style={{position:"fixed",inset:0,zIndex:200,display:"flex",alignItems:"center",justifyContent:"center"}}>
                <div onClick={function(){setChecklistOpen(false);}} style={{position:"absolute",inset:0,background:"rgba(0,0,0,0.6)",backdropFilter:"blur(4px)"}}/>
                <div style={{position:"relative",width:640,maxHeight:"80vh",overflowY:"auto",background:T.bgPanel,border:"1px solid "+T.accent+"44",borderRadius:8,boxShadow:"0 8px 32px rgba(0,0,0,0.4)",animation:"fadeUp 0.15s ease-out"}}>
                  <div style={{padding:"14px 18px",borderBottom:"1px solid "+T.border,display:"flex",alignItems:"center",justifyContent:"space-between"}}>
                    <div>
                      <h3 style={{fontFamily:SANS,fontSize:15,fontWeight:700,color:T.strong}}>Pre-Flight Checklist</h3>
                      <span style={{fontFamily:MONO,fontSize:9,color:T.muted}}>GO / NO-GO checks with overrides</span>
                    </div>
                    <button onClick={function(){setChecklistOpen(false);}} style={{fontFamily:MONO,fontSize:14,border:"none",background:"transparent",color:T.muted,cursor:"pointer",padding:"4px 8px"}}>✕</button>
                  </div>
                  <div style={{padding:18,display:"flex",flexDirection:"column",gap:8}}>
                    {checkResults.map(function(r) {
                      return (
                        <div key={r.id} style={{display:"flex",alignItems:"center",justifyContent:"space-between",padding:"10px 12px",borderRadius:4,border:"1px solid " + (r.pass ? T.accent + "33" : r.overridden ? T.warn + "33" : T.danger + "33"),background:r.pass ? T.accent + "08" : r.overridden ? T.warn + "08" : T.danger + "08"}}>
                          <div style={{display:"flex",alignItems:"center",gap:10}}>
                            <span style={{fontFamily:MONO,fontSize:12,fontWeight:700,color:r.pass ? T.accent : r.overridden ? T.warn : T.danger,minWidth:52}}>{r.status}</span>
                            <div>
                              <div style={{fontFamily:SANS,fontSize:12,fontWeight:600,color:T.strong}}>{r.label}</div>
                              <div style={{fontFamily:MONO,fontSize:9,color:T.muted,marginTop:1}}>{r.desc(checkCfg)}{r.configKey && (
                                <span> <input className="cfg" value={checkCfg[r.configKey]} onChange={function(e){var v=e.target.value;setCheckCfg(function(p){var n={...p};n[r.configKey]=v;return n;});}} style={{width:45,display:"inline",padding:"1px 4px",fontSize:9,marginLeft:4}} />{r.unit}</span>
                              )}</div>
                            </div>
                          </div>
                          {!r.pass && (
                            <button onClick={function(){toggleOverride(r.id);}} style={{fontFamily:MONO,fontSize:9,fontWeight:700,padding:"4px 12px",borderRadius:3,border:"1px solid "+(r.overridden?T.warn:T.danger),background:r.overridden?T.warn+"22":"transparent",color:r.overridden?T.warn:T.danger,cursor:"pointer"}}>{r.overridden?"OVRD ✓":"OVERRIDE"}</button>
                          )}
                        </div>
                      );
                    })}
                    {/* Summary */}
                    <div style={{marginTop:8,padding:"12px 14px",borderRadius:5,background:allGo?T.accent+"12":T.danger+"12",border:"1px solid "+(allGo?T.accent+"44":T.danger+"44"),textAlign:"center"}}>
                      <span style={{fontFamily:COND,fontSize:13,fontWeight:700,letterSpacing:2,color:allGo?T.accent:T.danger}}>{allGo?"● CASPER GO FOR LAUNCH":"● NO-GO — RESOLVE ITEMS"}</span>
                    </div>
                  </div>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </ThemeCtx.Provider>
  );
}