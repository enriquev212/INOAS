function [T_abs_to_ref, T_ref_to_abs] = referenceFrameTransform(r_ref, v_ref)

    r_ref = r_ref(:);
    v_ref = v_ref(:);

    % Eje radial: desde la Tierra hacia el satélite de referencia
    e_radial = r_ref / norm(r_ref);

    % Eje normal: perpendicular al plano orbital
    e_normal = cross(r_ref, v_ref);
    e_normal = e_normal / norm(e_normal);

    % Eje tangencial: dirección de avance orbital
    e_tangential = cross(e_normal, e_radial);
    e_tangential = e_tangential / norm(e_tangential);

    % Pasa vectores absolutos a ejes locales de referencia
    T_abs_to_ref = [e_radial.';
                    e_tangential.';
                    e_normal.'];

    % Pasa vectores locales de referencia a absolutos
    T_ref_to_abs = T_abs_to_ref.';
end