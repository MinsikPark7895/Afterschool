import React from "react";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import MainPage from "./pages/MainPage";
import LoginPage from "./pages/LoginPage";
import MapPage from "./pages/MapPage";
import AdminUserPage from "./pages/AdminUserPage";
// import RobotListPage from "./pages/RobotListPage";
// import RobotDetailPage from "./pages/RobotDetailPage";
import AdminDashboardPage from "./pages/AdminDashboardPage";
import PatrolModePage from "./pages/PatrolModePage";
import EventPage from "./pages/EventPage";
import ProtectedRoute from "./components/Auth/ProtectedRoute";

function App() {
  return (
    <Router
      future={{
        v7_startTransition: true,
        v7_relativeSplatPath: true,
      }}
    >
      <div className="App">
        <Routes>
          <Route path="/" element={<LoginPage />} />
          <Route 
            path="/main" 
            element={
              <ProtectedRoute>
                <MainPage />
              </ProtectedRoute>
            } 
          />
          <Route 
            path="/map" 
            element={
              <ProtectedRoute>
                <MapPage />
              </ProtectedRoute>
            } 
          />
          <Route 
            path="/admin/users" 
            element={
              <ProtectedRoute adminOnly>
                <AdminUserPage />
              </ProtectedRoute>
            } 
          />
          {/* <Route path="/robots" element={<RobotListPage />} /> */}
          {/* <Route path="/robots/:robotId" element={<RobotDetailPage />} /> */}
          <Route 
            path="/admin/dashboard" 
            element={
              <ProtectedRoute adminOnly>
                <AdminDashboardPage />
              </ProtectedRoute>
            } 
          />
          <Route 
            path="/patrol" 
            element={
              <ProtectedRoute>
                <PatrolModePage />
              </ProtectedRoute>
            } 
          />
          <Route 
            path="/events" 
            element={
              <ProtectedRoute>
                <EventPage />
              </ProtectedRoute>
            } 
          />
        </Routes>
      </div>
    </Router>
  );
}

export default App;
