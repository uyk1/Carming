package com.carming.backend.login.authentication;

import com.sun.security.auth.UserPrincipal;
import io.jsonwebtoken.*;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.userdetails.User;
import org.springframework.stereotype.Component;

import javax.servlet.http.HttpServletRequest;
import java.util.Date;

@Slf4j
@Component
public class JwtProvider {

    //todo @Value()
    private String jwtSecret = "secretKey";

    //todo @Value()
    private int jwtExpirationTime = 3600_000;



    public String generateToken(Authentication authentication) {
        User userPrincipal = (User) authentication.getPrincipal();
        Date now = new Date();
        Date expiryDate = new Date(now.getTime() + jwtExpirationTime);

        return Jwts.builder()
                .setSubject(userPrincipal.getUsername())
                .setIssuedAt(now)
                .setExpiration(expiryDate)
                .signWith(SignatureAlgorithm.HS512, jwtSecret)
                .compact();
    }


    public boolean validToken(String token) {
        try {
            Jwts.parser().setSigningKey(jwtSecret).parseClaimsJws(token);
            return true;
        } catch (JwtException e) {
            e.printStackTrace();
        }
        return false;
    }

    public String getUserInfoFromToken(String token) {
        try {
            return Jwts.parser().setSigningKey(jwtSecret).parseClaimsJws(token)
                    .getBody()
                    .getSubject();
        } catch (ExpiredJwtException e) {
            log.info("token Expired");
            return e.getClaims().getSubject();
        }
    }

    public String resolveToken(HttpServletRequest request) {
        String bearerToken = request.getHeader(JwtConst.HEADER_STRING);
        if (bearerToken != null && bearerToken.startsWith(JwtConst.TOKEN_TYPE)) {
            return bearerToken.substring(7);
        }
        return null;
    }

}
