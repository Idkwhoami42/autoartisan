import { ThemeProvider } from "~/components/theme-provider";
import { BrowserRouter, Routes, Route, useNavigate } from "react-router-dom";
import { Button } from "./components/ui/button";
import Dashboard from "./dashboard";

const App = () => {
  return (
    <ThemeProvider defaultTheme="dark" storageKey="vite-ui-theme">
      <BrowserRouter>
        <Routes>
          <Route path="/" element={<AppC />} />
          <Route path="/dashboard" element={<Dashboard />} />
        </Routes>
      </BrowserRouter>
    </ThemeProvider>
  );
};

const AppC = () => {
  const navigate = useNavigate();
  return (
    <main className="min-h-screen flex flex-col items-center justify-center font-mono gap-4">
      <h1 className="text-4xl font-bold">MASON</h1>
      <Button variant="secondary" onClick={() => navigate("/dashboard")}>
        Dashboard
      </Button>
    </main>
  );
};

export default App;
