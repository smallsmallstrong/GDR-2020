function angle_normalized = normalizeAngles(angle)
  % normalize to [-pi, pi];
  angle_normalized = mod(mod(angle, 2.0*pi) + 2.0*pi, 2.0*pi);
  
  for i = 1:length(angle_normalized)
    if (angle_normalized(i) > pi)
      angle_normalized(i) -= 2.0 *pi;
    end
  end
end