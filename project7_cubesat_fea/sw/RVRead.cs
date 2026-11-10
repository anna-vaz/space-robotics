// Reads a solved random-vibration study (no solve) and prints RMS results in several components/units,
// to pin down what GetMinMaxStressRMS / GetMinMaxDisplacementRMS return. Env: RR_STUDY.
using System;
using System.Linq;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class RVRead
{
    static string F(object o)
    {
        if (o == null || o is DBNull) return "null";
        return string.Join(", ", ((Array)o).Cast<object>().Select(x => Convert.ToDouble(x).ToString("G5")).ToArray());
    }
    public static void Run(string outDir)
    {
        var sw = SwSim.Connect(Console.Out);
        var doc = (ModelDoc2)sw.ActiveDoc;
        var cw = SwSim.Simulation(sw, Console.Out);
        var sm = cw.ActiveDoc.StudyManager;
        string want = System.Environment.GetEnvironmentVariable("RR_STUDY");
        for (int i = 0; i < sm.StudyCount; i++)
        {
            var st = sm.GetStudy(i);
            if (st == null || st.Name != want) continue;
            var r = st.Results; int err;
            if (r == null) { Console.WriteLine("no results"); return; }
            foreach (int u in new[] { 0, 3 })
                foreach (int c in new[] { 0, 1, 2, 9 })
                    Console.WriteLine("stressRMS comp " + c + " unit " + u + ": " + F(r.GetMinMaxStressRMS(c, 0, null, u, out err)) + " err " + err);
            foreach (int u in new[] { 0, 2 })
                foreach (int c in new[] { 0, 1, 2, 3 })
                    Console.WriteLine("dispRMS comp " + c + " unit " + u + ": " + F(r.GetMinMaxDisplacementRMS(c, null, u, out err)) + " err " + err);
            foreach (int c in new[] { 0, 1, 2, 3 })
                foreach (int step in new[] { 0, 1 })
                {
                    object a = null; try { a = r.GetMinMaxAcceleration(c, step, null, 3, out err); } catch (Exception ex) { Console.WriteLine("acc ex " + ex.Message); continue; }
                    Console.WriteLine("acc comp " + c + " step " + step + " (g): " + F(a) + " err " + err);
                }
            double x, y, z;
            foreach (int n in new[] { 81041, 86044, 86038 }) { st.Mesh.GetNodeLocation(n, out x, out y, out z); Console.WriteLine("node " + n + " at (" + (x * 1e3).ToString("F1") + ", " + (y * 1e3).ToString("F1") + ", " + (z * 1e3).ToString("F1") + ") mm"); }
        }
    }
}
