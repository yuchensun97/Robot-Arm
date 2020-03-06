function side_bool = detect_same_side(A,B,C)
    side_bool = (C(2)-A(2)) * (B(1)-A(1)) > (B(2)-A(2)) * (C(1)-A(1));
end