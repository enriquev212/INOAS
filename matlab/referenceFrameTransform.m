function [T_abs_to_ref, T_ref_to_abs] = referenceFrameTransform(r_ref, v_ref)

    r_ref = r_ref(:);
    v_ref = v_ref(:);

    % Radial axis: from Earth toward the reference spacecraft.
    e_radial = r_ref / norm(r_ref);

    % Normal axis: perpendicular to the orbital plane.
    e_normal = cross(r_ref, v_ref);
    e_normal = e_normal / norm(e_normal);

    % Tangential axis: along the orbital motion direction.
    e_tangential = cross(e_normal, e_radial);
    e_tangential = e_tangential / norm(e_tangential);

    % Transform absolute vectors into the local reference axes.
    T_abs_to_ref = [e_radial.';
                    e_tangential.';
                    e_normal.'];

    % Transform local reference vectors back into the absolute frame.
    T_ref_to_abs = T_abs_to_ref.';
end
