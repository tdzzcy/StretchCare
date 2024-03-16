import React, { useState, useEffect } from 'react';
import './App.css';

function App() {
  const [fileContent, setFileContent] = useState('');
  const [temperatureLog, setTemperatureLog] = useState({})
  const [currentPatient, setCurrentPatient] = useState("Patient 1");
  const [temperatureLogs, setTemperatureLogs] = useState({
    "Patient 1": [],
    "Patient 2": ["17:00:21 36.3", "17:03:54 36.8", "17:05:48 36.5"],
    "Patient 3": ["17:01:25 36.9", "17:08:16 37.0", "17:10:26 36.8"],
  });
  const patients = Object.keys(temperatureLogs);

  useEffect(() => {
    // Define the function that fetches the file content
    const fetchFileContent = () => {
      fetch('http://10.18.57.181:5000/get-file-content') // Make sure the URL matches your Flask app's address
        .then(response => response.json())
        .then(data => write_data(data))
        .catch(error => setFileContent("Error reading temperature."));
    };

    const write_data = (data) => {
      setFileContent(data.content);
      setTemperatureLogs(prevLogs => {
        const newLogs = { ...prevLogs };
        const currentLog = newLogs[currentPatient];
        // Check if the new data content is different from the last entry for the current patient
        if (!currentLog.length || data.content.substring(0, 8) !== currentLog[currentLog.length - 1].substring(0, 8)) {
          newLogs[currentPatient] = [...currentLog, data.content];
        }
        return newLogs;
      });
    };

    // Call the function immediately to fetch the content the first time
    fetchFileContent();

    // Set up an interval to fetch the content every n seconds
    const intervalId = setInterval(fetchFileContent, 4000);

    // Clear the interval when the component unmounts
    return () => clearInterval(intervalId);
  }, [currentPatient]); // Empty dependency array means this effect runs only once on mount

  const handlePatientChange = (event) => {
    setCurrentPatient(event.target.value);
  };

  return (
    <div className="App">
      <header style={{ backgroundColor: '#f0f0f0' }}>
        <h1>Hello robot temperature measurement</h1>
      </header>
      <main>
        <pre style={{ fontSize: '1.25em' }}>{fileContent}</pre>
        <br></br>
        <div>
          <label htmlFor="patient-select">Choose a patient: </label>
          <br></br>
          <select id="patient-select" value={currentPatient} onChange={handlePatientChange}>
            <option value="">Select a patient</option>
            {patients.map((patient, index) => (
              <option key={index} value={patient}>{patient}</option>
            ))}
          </select>
        </div>

        <div>
          <ul>
            {temperatureLogs[currentPatient].slice(1).map((log, index) => (
              <li key={index}>{log}</li>
            ))}
          </ul>
        </div>
      </main>
    </div>
  );
}

export default App;
