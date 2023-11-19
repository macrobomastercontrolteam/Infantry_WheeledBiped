syms l1 l4 L0 angle0 angle3 angle1 angle2 angle4
A = sym('A', [2 2]);
A(1,1) = l1 * cos(angle0 + angle3) * sin(angle1 - angle2) / sin(angle2 - angle3);
A(1,2)=l1 * sin(angle0 + angle3) * sin(angle1 - angle2) / (L0 * sin(angle2 - angle3));
A(2,1)=l4 * cos(angle0 + angle2) * sin(angle3 - angle4) / sin(angle2 - angle3);
A(2,2)=l4 * sin(angle0 + angle2) * sin(angle3 - angle4) / (L0 * sin(angle2 - angle3));

B = sym('B', [2 2]);
B(1,1)=sin(angle0 + angle2)/(l1*sin(angle1 - angle2));
B(1,2)=-sin(angle0 + angle3)/(l4*sin(angle3 - angle4));
B(2,1)=-(L0*cos(angle0 + angle2))/(l1*sin(angle1 - angle2));
B(2,2)=(L0*cos(angle0 + angle3))/(l4*sin(angle3 - angle4));

simplify(A*B) % should be eye(2)