import logo from './logo.svg';
import './App.css';
import ROSLIB from 'roslib';

var ros;
var cmdVelTopic;
function App() {
  return (
    <div>
    <button onClick={connect}>connect</button>
    <button onClick={makeTopic}>MKTPC</button>
    <button onClick={movetest}>move</button>
    

<div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', height: '100vh' }}>
<button onClick={move}>Forward</button>
<div style={{ display: 'flex', flexDirection: 'row' }}>
  <button onClick={(move)}>Left</button>
  <button onClick={move}>Stop</button>
  <button onClick={move}>Right</button>
</div>
<button onClick={move}>Backward</button>
</div>


<div style={{ position: 'fixed', bottom: '10px', right: '200px', width: '500px'}}>
      <button onClick={move}>Outstretch</button>
      <div style={{ display: 'flex', flexDirection: 'row' }}>
      <button onClick={move}>Rotright</button>
      <button onClick={move}>Stop</button>
      <button onClick={move}>Rotleft</button>
      </div>
      <button onClick={move}>Shrink</button>
    </div>

</div>
  );
}

function connect(){
    ros = new ROSLIB.Ros({
    url: 'ws://slinky.hcrlab.cs.washington.edu:9090'
  })
}

function makeTopic(){
  cmdVelTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/stretch/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});
}

function movetest(){
  var twist = new ROSLIB.Message({linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}});
  cmdVelTopic.publish(twist);
}

function move(){
  var twist = new ROSLIB.Message({
    linear : {
    x : 1,
    y:0.2,
    z : 0.3
    },
    angular : {
    x : -0.1,
    y : -0.2,
     z : -0.3
    }
    });
  cmdVelTopic.publish(twist);
}

export default App;
