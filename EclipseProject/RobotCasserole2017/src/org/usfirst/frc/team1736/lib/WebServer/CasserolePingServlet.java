package org.usfirst.frc.team1736.lib.WebServer;

import java.io.IOException;
import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;


class CasserolePingServlet extends HttpServlet {

    /**
	 * This prevents warnings. I do not know why. Eclipse did this. Not me.
	 */
	private static final long serialVersionUID = -9076510709468876590L;

	@Override
    public void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
        response.setContentType("application/json;charset=utf-8");
        response.setStatus(HttpServletResponse.SC_OK);
        response.setHeader("Access-Control-Allow-Origin", "*");
        response.getWriter().println("\"pong\"");

    }

}
