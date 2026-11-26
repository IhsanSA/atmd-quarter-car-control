#!/usr/bin/env node
// Independent numerical audit of the saved Python notebook (26 Nov 2026).
// Main study: ATMD_Quarter_Car_Colab.ipynb. No external dependencies.
import { readFileSync } from "node:fs";

function runVerification(nb, fullTuning = false) {
  const core = (() => {
const mat=(n,m,val=0)=>Array.from({length:n},()=>Array(m).fill(val));
const eye=n=>mat(n,n).map((r,i)=>r.map((x,j)=>+(i===j)));
const tr=a=>a[0].map((_,j)=>a.map(r=>r[j]));
const mm=(a,b)=>a.map(r=>b[0].map((_,j)=>r.reduce((s,x,k)=>s+x*b[k][j],0)));
const add=(a,b)=>a.map((r,i)=>r.map((x,j)=>x+b[i][j]));
const scale=(a,s)=>a.map(r=>r.map(x=>s*x));
const maxabs=a=>Math.max(...a.flat().map(Math.abs));
const gauss=(A,b)=>{
 const n=b.length, a=A.map((r,i)=>[...r,b[i]]);
 for(let k=0;k<n;k++){
   let p=k;for(let i=k+1;i<n;i++)if(Math.abs(a[i][k])>Math.abs(a[p][k]))p=i;
   if(Math.abs(a[p][k])<1e-18)throw Error("singular system at "+k);
   [a[k],a[p]]=[a[p],a[k]];
   for(let i=k+1;i<n;i++){
     const v=a[i][k]/a[k][k];if(v===0)continue;
     for(let j=k+1;j<=n;j++)a[i][j]-=v*a[k][j];a[i][k]=0;
   }
 }
 let x=Array(n).fill(0);for(let i=n-1;i>=0;i--){let v=a[i][n];for(let j=i+1;j<n;j++)v-=a[i][j]*x[j];x[i]=v/a[i][i]}
 return x;
};
const expm=(a)=>{
 let mx=Math.max(...a.map(r=>r.reduce((s,x)=>s+Math.abs(x),0)));
 const power=Math.max(0,Math.ceil(Math.log2(mx/.25))), sa=scale(a,2**(-power)),n=a.length;
 let ans=eye(n),term=eye(n);
 for(let k=1;k<=64;k++){term=scale(mm(term,sa),1/k);ans=add(ans,term);if(maxabs(term)<1e-18)break}
 for(let i=0;i<power;i++)ans=mm(ans,ans);return ans;
};
const car={ms:300,mu:45,ks:18990,cs:1800,kt:195900,ct:0,ma:12};
const h={peakForce:255,continuousForce:62,halfTravel:.0475,stopOnset:.04,stiffness:200000,damping:850,transitionLength:.001,speed:2.5,eta:.95};
const ctrlDt=.002,dt=.0005;
const aa=car.ks/car.ms,dd=(car.ks+car.kt)/car.mu,bb=-car.ks/Math.sqrt(car.ms*car.mu);
const wh=Math.sqrt((aa+dd-Math.sqrt((aa-dd)**2+4*bb**2))/2);
const tuned={ka:car.ma*(wh*.84)**2,ca:2*(.06+5*(.35-.06)/15)*car.ma*wh*.84};
const state=(v,attached=true)=>{
 const ms=attached?[v.ms,v.mu,v.ma]:[v.ms,v.mu],n=ms.length, A=mat(n*2,n*2),B=mat(n*2,3);
 for(let i=0;i<n;i++)A[i][n+i]=1;
 const connection=(i,j,k,c)=>{
  A[n+i][i]-=k/ms[i];A[n+i][j]+=k/ms[i];A[n+j][i]+=k/ms[j];A[n+j][j]-=k/ms[j];
  A[n+i][n+i]-=c/ms[i];A[n+i][n+j]+=c/ms[i];A[n+j][n+i]+=c/ms[j];A[n+j][n+j]-=c/ms[j];
 };
 connection(0,1,v.ks,v.cs);if(attached)connection(0,2,tuned.ka,tuned.ca);
 A[n+1][1]-=v.kt/ms[1];A[n+1][n+1]-=v.ct/ms[1];B[n+1][0]=v.kt/ms[1];B[n+1][1]=v.ct/ms[1];
 if(attached){B[n][2]=1/ms[0];B[n+2][2]=-1/ms[2]}
 return {A,B};
};
const lyap=(F,S)=>{
 const n=F.length, N=n*n, eq=mat(N,N),rhs=[];
 for(let i=0;i<n;i++)for(let j=0;j<n;j++){
  const row=i*n+j;rhs[row]=-S[i][j];
  for(let k=0;k<n;k++){eq[row][k*n+j]+=F[k][i];eq[row][i*n+k]+=F[k][j]}
 }
 const sol=gauss(eq,rhs);return Array.from({length:n},(_,i)=>sol.slice(i*n,(i+1)*n));
};
const forceScale=Array.from({length:24},(_,i)=>2*(250/2)**(i/23)).reduce((a,b)=>Math.abs(a-70.9447)<Math.abs(b-70.9447)?a:b);
const lqr=(fs=forceScale,w=.02)=>{
 const {A,B}=state(car), b=B.map(r=>r[2]),n=A.length;
 const outputs=[[100,0,0,0,0,0],[0,0,0,1/.15,0,0],[-25,0,25,0,0,0],[0,0,0,-2,0,2]];
 const weights=[1,.20,w,.025],Q=mat(n,n);
 for(let i=0;i<n;i++)for(let j=0;j<n;j++)Q[i][j]=outputs.reduce((s,r,k)=>s+r[i]*r[j]*weights[k],0)+(i===j?1e-9:0);
 const R=1/fs**2;
 let K=Array(n).fill(0),P,iterations;
 for(let iter=0;iter<100;iter++){
  const F=A.map((r,i)=>r.map((x,j)=>x-b[i]*K[j])),S=Q.map((r,i)=>r.map((x,j)=>x+R*K[i]*K[j]));
  P=lyap(F,S);const knew=Array(n).fill(0).map((_,j)=>b.reduce((s,x,i)=>s+x*P[i][j],0)/R);
  let err=Math.max(...knew.map((x,i)=>Math.abs(x-K[i])));K=knew;iterations=iter+1;if(err<1e-8)break;
 }
 let AP=mm(tr(A),P),PA=mm(P,A),PB=P.map(r=>r.reduce((s,x,i)=>s+x*b[i],0));
 let residual=maxabs(add(add(AP,PA),Q).map((r,i)=>r.map((x,j)=>x-PB[i]*PB[j]/R)));
 return {K,forceScale:fs,iterations,residual};
};
const gain=lqr();
const controls=[{id:"Standard",attached:false,K:[0,0,0,0],Ki:0,beta:0},{id:"TMD",attached:true,K:Array(6).fill(0),Ki:0,beta:0},{id:"PID",attached:true,K:[800,0,0,35,0,0],Ki:96,beta:0},{id:"LQR",attached:true,K:gain.K,Ki:0,beta:0}];
const plant=(v,attached,step)=>{
 const {A,B}=state(v,attached),n=A.length,aug=mat(n+3,n+3);
 for(let i=0;i<n;i++){for(let j=0;j<n;j++)aug[i][j]=A[i][j]*step;for(let j=0;j<3;j++)aug[i][n+j]=B[i][j]*step}
 const p=expm(aug);
 return {Phi:p.slice(0,n).map(r=>r.slice(0,n)),Gamma:p.slice(0,n).map(r=>r.slice(n))};
};
const interp=(arr,z)=>{if(z<0||z>arr.length-1)return 0;const j=Math.floor(z);if(j===arr.length-1)return arr[j];return arr[j]+(z-j)*(arr[j+1]-arr[j]);};
const detrend=elev=>{
 return elev.map((y,i)=>{
  let first=Math.max(0,Math.min(elev.length-21,i-10)),a=mat(3,3),b=Array(3).fill(0);
  for(let j=first;j<first+21;j++){let row=[1,j-i,(j-i)**2];for(let k=0;k<3;k++){b[k]+=row[k]*elev[j];for(let l=0;l<3;l++)a[k][l]+=row[k]*row[l];}}
  return (y-gauss(a,b)[0])*.5*(1-Math.cos(Math.PI*Math.min(1,i/8,(elev.length-1-i)/8)));
 });
};
const scenario=(name,type,amplitude,length,speed,massFactor=1,duration=8,onset=.5)=>({name,type,amplitude,length,speed,massFactor,duration,onset});
const scenarios=[scenario("Smooth bump","bump",.022,2.5,10),scenario("Nominal sine","sine",.004,10,12),scenario("Sprung mass +25%","sine",.004,10,12,1.25),scenario("Sprung mass −20%","sine",.004,10,12,.80),scenario("Low frequency","sine",.004,10,8),scenario("High frequency","sine",.004,10,16),scenario("High mass and frequency","sine",.004,10,16,1.25),scenario("High-amplitude sine","sine",.030,10,12),scenario("ARAN section","measured",0,96,12,1,10)];
const road=(s,t,profile)=>{
 const z=(t-s.onset)*s.speed/s.length;
 if(s.type==="bump")return .5*s.amplitude*(1-Math.cos(2*Math.PI*Math.max(0,Math.min(1,z))));
 if(s.type==="sine")return s.amplitude*.5*(1-Math.cos(Math.PI*Math.max(0,Math.min(1,z))))*Math.sin(2*Math.PI*z);
 return interp(profile,(t-s.onset)*s.speed);
};
const simulate=(s,c,opts={})=>{
 const step=opts.step??dt,limits={...h,...opts.hardware},v={...car,...opts.car};v.ms*=s.massFactor??1;
 const attached=c.attached, {Phi,Gamma}=opts.plant??plant(v,attached,step),n=Phi.length,N=Math.round(s.duration/step),stride=Math.round(ctrlDt/step);
 const from=s.type==="sine"?Math.max(s.onset+s.length/s.speed,s.duration-3*s.length/s.speed):s.onset;
 const firstRms=Math.ceil((from-1e-10)/step),firstDuty=Math.ceil((s.onset-1e-10)/step);
 let x=Array(n).fill(0),next=Array(n).fill(0),integ=0,force=0,command=0,prevBody=0,prevForce=0,prevMotoring=0,prevAbsorbed=0;
 let peak=0,peakForce=0,maxTravel=0,maxContact=0,maxSpeed=0,sumBody=0,sumForce=0,sumDuty=0,events=0,prevContact=false,sat=0,normalMin=Infinity,energy=0,absorbed=0,peakElectrical=0;
 const history=[];let roadNow=road(s,0,opts.profile);
 for(let k=0;k<=N;k++){
  const body=x[0],wheel=x[1],travel=attached?x[2]-body:0,vel=attached?x[5]-x[3]:0;
  if(attached&&k%stride===0){command=-c.K.reduce((a,g,i)=>a+g*x[i],0)+integ;force=opts.forceLimit===false?command:Math.max(-limits.peakForce,Math.min(limits.peakForce,command));
   if(c.Ki>0)integ+=ctrlDt*(c.Ki*(-body+c.beta*travel)+(force-command)/.075-.15*integ)}
  let contact=0;
  if(attached&&opts.stops!==false&&Math.abs(travel)>limits.stopOnset){let depth=Math.abs(travel)-limits.stopOnset,dir=Math.sign(travel);
   contact=dir*(limits.stiffness*depth+limits.damping*Math.max(dir*vel,0)*Math.min(depth/limits.transitionLength,1));}
  if(contact!==0&&!prevContact)events++;prevContact=contact!==0;
  peak=Math.max(peak,Math.abs(body));peakForce=Math.max(peakForce,Math.abs(force));maxTravel=Math.max(maxTravel,Math.abs(travel));maxSpeed=Math.max(maxSpeed,Math.abs(vel));maxContact=Math.max(maxContact,Math.abs(contact));
  normalMin=Math.min(normalMin,9.80665*(v.ms+v.mu+(attached?v.ma:0))+v.kt*(roadNow-wheel));
  if(k>=firstDuty&&Math.abs(force-command)>1e-7)sat++;
  if(k>firstRms){sumBody+=.5*(body**2+prevBody**2);sumForce+=.5*(force**2+prevForce**2)}
  if(k>firstDuty)sumDuty+=.5*(force**2+prevForce**2);
  let power=-force*vel,motoring=Math.max(power,0)/limits.eta,absorb=Math.max(-power,0);peakElectrical=Math.max(peakElectrical,motoring);
  if(k>0){energy+=.5*(motoring+prevMotoring)*step;absorbed+=.5*(absorb+prevAbsorbed)*step}
  if(opts.trace&&(k%Math.round(.01/step)===0||k===N))history.push([k*step,1000*roadNow,1000*body,force,1000*travel]);
  prevForce=force;prevBody=body;prevMotoring=motoring;prevAbsorbed=absorb;
  if(k===N)break;
  const roadNext=road(s,(k+1)*step,opts.profile),avg=.5*(roadNow+roadNext),slope=(roadNext-roadNow)/step;
  for(let i=0;i<n;i++){
   let val=0;for(let j=0;j<n;j++)val+=x[j]*Phi[i][j];
   next[i]=val+avg*Gamma[i][0]+slope*Gamma[i][1]+(force+contact)*Gamma[i][2];
  }
  [x,next]=[next,x];roadNow=roadNext;
 }
 const rmsForce=Math.sqrt(sumForce/(N-firstRms)),dutyRms=Math.sqrt(sumDuty/(N-firstDuty));
 return {id:c.id,peak_mm:peak*1000,rms_mm:1000*Math.sqrt(sumBody/(N-firstRms)),rms_force_N:rmsForce,peak_force_N:peakForce,duty_rms_N:dutyRms,stroke_mm:attached?1000*maxTravel:null,contact_events:events,contact_peak_N:maxContact,speed_m_s:maxSpeed,sat_pct:100*sat/(N-firstDuty+1),normal_min_N:normalMin,motor_energy_J:energy,absorbed_energy_J:absorbed,peak_electrical_W:peakElectrical,screen:(!attached||maxTravel<=limits.halfTravel)&&normalMin>=0&&maxSpeed<=limits.speed&&Math.max(rmsForce,dutyRms)<=limits.continuousForce+1e-7,rms_from_s:firstRms*step,rms_to_s:N*step,history:opts.trace?history:undefined};
};
return {mat,eye,tr,mm,add,scale,gauss,expm,car,h,tuned,wh,forceScale,gain,lqr,state,plant,controls,scenario,scenarios,road,detrend,simulate,dt,ctrlDt};
})();
  // Exact embedded sample; the original Python notebook is the canonical model.
  const source = c => Array.isArray(c.source) ? c.source.join("") : c.source;
  const measured = nb.cells.find(c => source(c).includes("embedded_elevation = np.array(["));
  const elevation = source(measured).match(/embedded_elevation = np\.array\(\[([\s\S]*?)\]\)/)[1]
    .split(",").map(x => parseFloat(x)).filter(Number.isFinite);
  const profile = core.detrend(elevation);
  const evaluation = core.scenarios.map(s => {
    const results = core.controls.map(c => core.simulate(s, c, {profile}));
    const tmd = results.find(r => r.id === "TMD");
    for (const r of results) r.extra_pct = 100 * (1 - r.peak_mm / tmd.peak_mm);
    return {name: s.name, results};
  });

  // Compare against the two-decimal table displayed in the saved notebook.
  const tableCell = nb.cells.find(c => source(c).includes("evaluation_rows = {},[]"));
  const rawHtml = tableCell.outputs[0].data["text/html"];
  const html = Array.isArray(rawHtml) ? rawHtml.join("") : rawHtml;
  const rows = Array.from(html.matchAll(/<tr>\s*(<td>[\s\S]*?)<\/tr>/g),
    m => Array.from(m[1].matchAll(/<td>(.*?)<\/td>/gs), x => x[1]));
  const keys = ["peak_mm","rms_mm","extra_pct","peak_force_N","rms_force_N","stroke_mm",
    "contact_events","screen","rms_from_s","rms_to_s"];
  const errors = [];
  let checks = 0;
  for (const row of rows) {
    const r = evaluation.find(s => s.name === row[0]).results.find(m => m.id === row[1]);
    for (let k = 0; k < keys.length; k++) {
      const actual = r[keys[k]], expected = row[k + 2];
      const ok = keys[k] === "screen" ? actual === (expected === "True")
        : expected === "—" ? actual === null
        : Math.abs(actual - Number(expected)) <= 0.00500001;
      checks++;
      if (!ok) errors.push({case: row[0], controller: row[1], metric: keys[k], expected, actual});
    }
  }
  if (rows.length !== 36 || checks !== 360 || errors.length)
    throw new Error("Saved-result comparison failed: " + JSON.stringify({rows: rows.length, checks, errors}));

  // Continuous linear road-to-body frequency response, via a real block solve
  // of (j omega I - A_closed) x = B_road.
  function frf(c, f) {
    const {A, B} = core.state(core.car, c.attached);
    const n = A.length, m = n + (c.Ki ? 1 : 0);
    const cl = core.mat(m, m), input = Array(m).fill(0);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) cl[i][j] = A[i][j] - B[i][2] * c.K[j];
      input[i] = B[i][0];
      if (c.Ki) cl[i][n] = B[i][2];
    }
    if (c.Ki) {
      cl[n][0] = -c.Ki * (1 + c.beta);
      cl[n][2] = c.Ki * c.beta;
      cl[n][n] = -0.15;
    }
    const w = 2 * Math.PI * f, M = core.mat(2*m, 2*m);
    for (let i = 0; i < m; i++) {
      for (let j = 0; j < m; j++) {
        M[i][j] = -cl[i][j];
        M[m+i][m+j] = -cl[i][j];
        if (i === j) { M[i][m+j] = -w; M[m+i][j] = w; }
      }
    }
    const x = core.gauss(M, [...input, ...Array(m).fill(0)]);
    return {gain: Math.hypot(x[0], x[m]), phase_deg: Math.atan2(x[m], x[0]) * 180 / Math.PI};
  }

  const bands = core.controls.slice(2).map(c => {
    const fn = f => frf(c, f).gain - frf(core.controls[1], f).gain;
    let edges = [0.55], left = 0.55, previous = fn(left);
    for (let k = 1; k <= 650; k++) {
      const right = 0.55 * (2.5 / 0.55) ** (k / 650), v = fn(right);
      if (previous * v < 0) {
        let l = left, r = right;
        for (let j = 0; j < 45; j++) {
          const mid = (l + r) / 2;
          if (fn(l) * fn(mid) <= 0) r = mid; else l = mid;
        }
        edges.push((l + r) / 2);
      }
      left = right; previous = v;
    }
    edges.push(2.5);
    const good = edges.slice(0, -1).map((a,i) => [a, edges[i+1]])
      .filter(([a,b]) => fn(Math.sqrt(a*b)) < 0);
    return {controller: c.id, intervals_hz: good};
  });
  const nominal = evaluation.find(s => s.name === "Nominal sine");
  const stress = evaluation.find(s => s.name === "High-amplitude sine");
  const refinement = [core.scenarios[1], core.scenarios[7]].map(s => ({
    scenario: s.name, results: core.controls.slice(1).map(c => {
      const a = evaluation.find(r => r.name === s.name).results.find(x => x.id === c.id);
      const b = core.simulate(s, c, {step: core.dt / 2});
      return {controller: c.id, peak_change_mm: b.peak_mm - a.peak_mm,
        stroke_change_mm: b.stroke_mm - a.stroke_mm, contacts_original: a.contact_events,
        contacts_refined: b.contact_events};
    })
  }));

  const report = {
    time_domain: {scenarios: 9, configurations: 4, trajectories: rows.length,
      displayed_fields_checked: checks, mismatches: errors.length},
    passive_design: {stiffness_N_m: core.tuned.ka, damping_Ns_m: core.tuned.ca},
    lqr_gain: core.gain,
    nominal: nominal.results,
    stress: stress.results,
    frequency_at_1p2_hz: core.controls.map(c => ({controller:c.id, ...frf(c, 1.2)})),
    relative_gain_bands: bands,
    half_step: refinement,
    scope: "Independent JavaScript numerical recheck, not execution of the full Python notebook. The Monte Carlo batch and saved reference-case checks were not rerun."
  };

  if (fullTuning) {
    // Passive minimax search. Work in the mechanical impedance domain.
    const v = core.car, f0 = core.wh / (2*Math.PI);
    function plantGain(f, ka, ca) {
      const mass = [v.ms,v.mu,v.ma];
      const k = [[v.ks+ka,-v.ks,-ka],[-v.ks,v.ks+v.kt,0],[-ka,0,ka]];
      const c = [[v.cs+ca,-v.cs,-ca],[-v.cs,v.cs+v.ct,0],[-ca,0,ca]];
      const w = 2*Math.PI*f, M = core.mat(6,6);
      for (let i=0;i<3;i++) for (let j=0;j<3;j++) {
        const re = k[i][j] - (i===j ? w*w*mass[i] : 0), im = w*c[i][j];
        M[i][j] = re; M[i][j+3] = -im;
        M[i+3][j] = im; M[i+3][j+3] = re;
      }
      const x = core.gauss(M, [0,v.kt,0,0,w*v.ct,0]);
      return Math.hypot(x[0],x[3]);
    }
    let best = {maxGain: Infinity};
    for (let i=0;i<=20;i++) for (let j=0;j<=15;j++) {
      const ratio=0.75+i*(1.05-0.75)/20, zeta=0.06+j*(0.35-0.06)/15;
      const ka=v.ma*(ratio*core.wh)**2, ca=2*zeta*v.ma*core.wh*ratio;
      let maxGain=0;
      for (let k=0;k<=120;k++) maxGain=Math.max(maxGain,plantGain(f0*(0.65+k*(1.5-0.65)/120),ka,ca));
      if (maxGain<best.maxGain) best={ratio,zeta,ka,ca,maxGain};
    }

    // The saved notebook has all 276 candidates pass nominal stability.
    // Re-evaluate this same candidate set; the independent audit does not rerun
    // the notebook's eigenvalue-based stability filter.
    const candidates = [];
    for (const kp of [0,800,2500,6000,12000]) for (const ir of [.12,.28,.65])
      for (const kd of [35,180,550,1150]) for (const beta of [0,.2,.7]) {
        const ki = Math.max(kp,100)*ir;
        candidates.push({id:"P"+String(candidates.length).padStart(3,"0"),family:"PID",
          attached:true,K:[kp*(1+beta),0,-kp*beta,kd*(1+beta),0,-kd*beta],
          Ki:ki,beta,parameters:[kp,ki,kd,beta]});
      }
    for (let i=0;i<24;i++) for (const w of [.02,.15,.8,3]) {
      const fs=2*(250/2)**(i/23), q=core.lqr(fs,w);
      candidates.push({id:"L"+String(candidates.length).padStart(3,"0"),family:"LQR",
        attached:true,K:q.K,Ki:0,beta:0,parameters:[fs,w]});
    }
    const training = [core.scenario("Training bump","bump",.012,3,10),
      core.scenario("Training sine","sine",.003,10,12)];
    const plant = core.plant(v,true,core.dt);
    const references=training.map(s=>core.simulate(s,core.controls[1],{plant}));
    const scores=candidates.map(c=>{
      const outcomes=training.map((s,j)=>{
        const m=core.simulate(s,c,{plant});
        return {gain:100*(1-m.peak_mm/references[j].peak_mm),
          force:m.rms_force_N,screen:m.screen,contacts:m.contact_events};
      });
      return {id:c.id,family:c.family,parameters:c.parameters,
        mean:outcomes.reduce((t,r)=>t+r.gain,0)/2,worst:Math.min(...outcomes.map(r=>r.gain)),
        force:outcomes.reduce((t,r)=>t+r.force,0)/2,valid:outcomes.every(r=>r.screen),
        contacts:Math.max(...outcomes.map(r=>r.contacts))};
    });
    const pool=scores.filter(s=>s.valid&&s.contacts===0&&s.worst>=-3);
    const p=pool.filter(s=>s.family==="PID"), l=pool.filter(s=>s.family==="LQR");
    const target=.8*Math.min(Math.max(...p.map(s=>s.mean)),Math.max(...l.map(s=>s.mean)));
    const pairs=p.flatMap(a=>l.map(b=>({PID:a,LQR:b,gap:Math.abs(a.mean-b.mean),
      common:Math.min(a.mean,b.mean),sum:a.force+b.force})));
    const eligible=pairs.filter(x=>x.gap<=1&&x.common>=Math.max(0,target));
    const selected=eligible.length?eligible.sort((a,b)=>a.sum-b.sum||a.gap-b.gap)[0]
      :pairs.sort((a,b)=>a.gap-b.gap||b.common-a.common)[0];
    report.tuning = {passive_designs:336,frequency_points_per_design:121,best,
      candidates:candidates.length,training_trajectories:candidates.length*2,
      selected,training_target_pct:target};
    if (selected.PID.id!=="P036" || selected.LQR.id!=="L248" || Math.abs(best.ratio-.84)>1e-9)
      throw new Error("Tuning selection differs from the saved notebook.");
  }
  return report;
}

const notebook = JSON.parse(readFileSync(new URL("../ATMD_Quarter_Car_Colab.ipynb", import.meta.url), "utf8"));
const report = runVerification(notebook, process.argv.includes("--tuning"));
console.log(JSON.stringify(report, null, 2));
