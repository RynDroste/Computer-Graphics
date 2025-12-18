 int A = (int)(X + Y * 57 + Z * 131) & 255;
        int B = (int)((X + 1) + Y * 57 + Z * 131) & 255;
        int AA = (int)(X + (Y + 1) * 57 + Z * 131) & 255;
        int BA = (int)((X + 1) + (Y + 1) * 57 + Z * 131) & 255;
        int AB = (int)(X + Y * 57 + (Z + 1) * 131) & 255;
        int BB = (int)((X + 1) + Y * 57 + (Z + 1) * 131) & 255;
        int AAB = (int)(X + (Y + 1) * 57 + (Z + 1) * 131) & 255;
        int BAB = (int)((X + 1) + (Y + 1) * 57 + (Z + 1) * 131) & 255;
        