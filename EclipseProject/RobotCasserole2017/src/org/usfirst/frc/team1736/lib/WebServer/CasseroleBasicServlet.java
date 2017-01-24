package org.usfirst.frc.team1736.lib.WebServer;

import java.io.IOException;
import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

class CasseroleBasicServlet extends HttpServlet {

    /**
	 * This prevents warnings. I do not know why. Eclipse did this. Not me.
	 */
	private static final long serialVersionUID = -5455816632333702006L;

	@Override
    public void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
        response.setContentType("text/html");
        response.setStatus(HttpServletResponse.SC_OK);
        response.getWriter().println("<h1>FRC1736 Robot Casserole 2016 </h1>");

    }

}
