using Elements;
using Elements.Geometry;
using System.Collections.Generic;
using Elements.G3;
using DotLiquid.Exceptions;

namespace FastProjectToTopography
{
  public static class FastProjectToTopography
  {
    /// <summary>
    /// The FastProjectToTopography function.
    /// </summary>
    /// <param name="model">The input model.</param>
    /// <param name="input">The arguments to the execution.</param>
    /// <returns>A FastProjectToTopographyOutputs instance containing computed results and the model with any new elements.</returns>
    public static FastProjectToTopographyOutputs Execute(Dictionary<string, Model> inputModels, FastProjectToTopographyInputs input)
    {
      // Your code here.
      var output = new FastProjectToTopographyOutputs();

      // Load model dependencies.
      var locationModel = inputModels["location"];
      var topo = locationModel.AllElementsOfType<Topography>().FirstOrDefault();
      if (topo is null)
      {
        output.Warnings.Add("Topo was null");
        return output;
      }
      // Gather inputs.
      var polygons = input.Polygons;

      var raycaster = new Raycaster(topo.Mesh, topo.Transform);
      var rayCount = 0;
      var stopwatch = new System.Diagnostics.Stopwatch();
      stopwatch.Start();
      foreach (var pgon in polygons)
      {
        var newVertices = new List<Vector3>();
        var pts = new List<Vector3>();
        var subdivisionLength = input.RayDensity;
        foreach (var segment in pgon.Segments())
        {
          var length = segment.Length();
          var count = (int)(length / subdivisionLength);
          for (int i = 0; i <= count; i++)
          {
            pts.Add(segment.PointAtNormalized(i / (double)count));
          }
        }
        var rays = pts.Select(pt => new Ray(pt + (0, 0, 1000), Vector3.ZAxis.Negate()));
        foreach (var ray in rays)
        {
          // Using the G3 fast raycaster
          if (input.UseG3)
          {
            var intersection = raycaster.Cast(ray);
            if (intersection.Hit)
            {
              newVertices.Add(intersection.HitPoint!.Value);
            }
            else
            {
              newVertices.Add(ray.Origin);
            }
          }
          else // Using Elements' Ray / Topo intersection
          {
            var intersects = ray.Intersects(topo, out var result);
            if (intersects)
            {
              newVertices.Add(result);
            }
            else
            {
              newVertices.Add(ray.Origin);
            }

          }
          rayCount++;
        }
        newVertices.Add(newVertices[0]);
        var pl = new Polyline(newVertices);
        output.Model.AddElement(new ModelCurve(pl, BuiltInMaterials.XAxis, new Transform(0, 0, 1)));
      }
      stopwatch.Stop();
      output.Warnings.Add($"Raycasted {rayCount} rays in {stopwatch.ElapsedMilliseconds}ms");

      return output;
    }
  }
}